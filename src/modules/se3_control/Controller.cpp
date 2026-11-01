// Geometric translational tracking.
//
// Position and velocity tracking errors:
//
//   e_x = x - x_d
//   e_v = v - v_d
//
//   a_cmd = a_d - (k_x/m)e_x - (k_v/m)e_v
//
// The desired physical force includes gravity compensation:
//
//   A = m(a_cmd - g e3)
//
// PX4 local NED has positive z downward. Therefore g e3 is positive
// along NED z, and hover produces a negative-z desired force.

#include "Controller.hpp"
#include "AttitudeReference.hpp"

#include <cmath>

namespace se3_control
{
namespace
{

constexpr float kGravity = 9.80665f;

bool finite(const trajectory_generator::Vector3 & vector)
{
  return std::isfinite(vector[0]) &&
         std::isfinite(vector[1]) &&
         std::isfinite(vector[2]);
}

}  // namespace

bool Controller::compute_translation(
  const State & state,
  const trajectory_generator::Reference & reference,
  const TranslationGains & gains,
  TranslationOutput & output)
{
  // Validate inputs before calculating a control command.
  if (!std::isfinite(gains.mass) || gains.mass <= 0.0f ||
      !std::isfinite(gains.position) || gains.position <= 0.0f ||
      !std::isfinite(gains.velocity) || gains.velocity <= 0.0f ||
      !state.position.isAllFinite() ||
      !state.velocity.isAllFinite() ||
      !finite(reference.position) ||
      !finite(reference.velocity) ||
      !finite(reference.acceleration) ||
      !std::isfinite(reference.yaw)) {
    return false;
  }

  const Vector3 desired_position{
    reference.position[0],
    reference.position[1],
    reference.position[2]
  };

  const Vector3 desired_velocity{
    reference.velocity[0],
    reference.velocity[1],
    reference.velocity[2]
  };

  const Vector3 desired_acceleration{
    reference.acceleration[0],
    reference.acceleration[1],
    reference.acceleration[2]
  };

  // Position and velocity tracking errors in local NED.
  const Vector3 position_error = state.position - desired_position;
  const Vector3 velocity_error = state.velocity - desired_velocity;

  // Kinematic acceleration command with trajectory feedforward.
  const Vector3 acceleration =
    desired_acceleration -
    position_error * (gains.position / gains.mass) -
    velocity_error * (gains.velocity / gains.mass);

  const Vector3 gravity{0.0f, 0.0f, kGravity};

  // Physical force reference used to construct the desired attitude
  // and collective thrust in subsequent controller calculations.
  const Vector3 force =
    (acceleration - gravity) * gains.mass;

  if (!acceleration.isAllFinite() || !force.isAllFinite()) {
    return false;
  }

  output.acceleration = acceleration;
  output.force = force;

  return true;
}

// Analytic desired-force derivatives from the nominal rigid-body model.
//
// Desired force:
//   A = -k_x e_x - k_v e_v - m g e3 + m a_d
//
// Model acceleration:
//   a_model = g e3 - (f/m)b3
//   f       = -A . b3
//
// Force derivatives:
//   A_dot  = m j_d - k_x e_v - k_v e_a
//   A_ddot = m s_d - k_x e_a - k_v e_j
//
// Translational acceleration and jerk are model-derived, not obtained by
// numerically differentiating estimator measurements.
bool Controller::compute_force_derivatives(
  const State & state,
  const trajectory_generator::Reference & reference,
  const TranslationGains & gains,
  const TranslationOutput & translation,
  ForceDerivatives & output)
{
  if (!std::isfinite(gains.mass) || gains.mass <= 0.0f ||
      !std::isfinite(gains.position) || gains.position <= 0.0f ||
      !std::isfinite(gains.velocity) || gains.velocity <= 0.0f ||
      !state.velocity.isAllFinite() ||
      !state.attitude.isAllFinite() ||
      !state.angular_velocity.isAllFinite() ||
      !translation.force.isAllFinite() ||
      !finite(reference.velocity) ||
      !finite(reference.acceleration) ||
      !finite(reference.jerk) ||
      !finite(reference.snap)) {
    return false;
  }

  const Vector3 velocity_reference{
    reference.velocity[0],
    reference.velocity[1],
    reference.velocity[2]
  };

  const Vector3 acceleration_reference{
    reference.acceleration[0],
    reference.acceleration[1],
    reference.acceleration[2]
  };

  const Vector3 jerk_reference{
    reference.jerk[0],
    reference.jerk[1],
    reference.jerk[2]
  };

  const Vector3 snap_reference{
    reference.snap[0],
    reference.snap[1],
    reference.snap[2]
  };

  const Vector3 gravity{0.0f, 0.0f, kGravity};

  // Current body z-axis expressed in local NED.
  const Vector3 body_z{
    state.attitude(0, 2),
    state.attitude(1, 2),
    state.attitude(2, 2)
  };

  const float thrust = -(translation.force * body_z);

  const Vector3 model_acceleration =
    gravity - body_z * (thrust / gains.mass);

  const Vector3 velocity_error =
    state.velocity - velocity_reference;

  const Vector3 acceleration_error =
    model_acceleration - acceleration_reference;

  // First force derivative from reference jerk and tracking errors.
  const Vector3 force_first =
    jerk_reference * gains.mass -
    velocity_error * gains.position -
    acceleration_error * gains.velocity;

  // R_dot = R hat(Omega).
  // Differentiate the body z-axis using measured body angular velocity.
  const Vector3 body_z_rate_body =
    state.angular_velocity.cross(Vector3{0.0f, 0.0f, 1.0f});

  const Vector3 body_z_first{
    state.attitude(0, 0) * body_z_rate_body(0) +
      state.attitude(0, 1) * body_z_rate_body(1) +
      state.attitude(0, 2) * body_z_rate_body(2),

    state.attitude(1, 0) * body_z_rate_body(0) +
      state.attitude(1, 1) * body_z_rate_body(1) +
      state.attitude(1, 2) * body_z_rate_body(2),

    state.attitude(2, 0) * body_z_rate_body(0) +
      state.attitude(2, 1) * body_z_rate_body(1) +
      state.attitude(2, 2) * body_z_rate_body(2)
  };

  // Differentiate f = -A . b3 analytically.
  const float thrust_first =
    -(force_first * body_z) -
    (translation.force * body_z_first);

  // Nominal rigid-body jerk from thrust and body-axis derivatives.
  const Vector3 model_jerk =
    -body_z * (thrust_first / gains.mass) -
    body_z_first * (thrust / gains.mass);

  const Vector3 jerk_error = model_jerk - jerk_reference;

  // Second desired-force derivative for angular acceleration feedforward.
  const Vector3 force_second =
    snap_reference * gains.mass -
    acceleration_error * gains.position -
    jerk_error * gains.velocity;

  if (!force_first.isAllFinite() || !force_second.isAllFinite()) {
    return false;
  }

  output.first = force_first;
  output.second = force_second;

  return true;
}

// Calculate the common SE(3) reference before applying either torque law.
//
// Translation generates the physical force A. Model-based force derivatives
// provide the analytic desired angular velocity and acceleration.
//
// The collective thrust follows Lee's projection onto the current body Z:
//
//   f = -A . (R e3)
//
// A non-positive collective command is rejected. The runtime must not
// publish an invalid thrust command to the control allocator.
bool Controller::compute_geometry(
  const State & state,
  const trajectory_generator::Reference & reference,
  const TranslationGains & gains,
  GeometricOutput & output)
{
  GeometricOutput candidate{};

  if (!compute_translation(
        state, reference, gains, candidate.translation)) {
    return false;
  }

  if (!compute_force_derivatives(
        state,
        reference,
        gains,
        candidate.translation,
        candidate.force_derivatives)) {
    return false;
  }

  if (!AttitudeReference::compute(
        candidate.translation.force,
        candidate.force_derivatives.first,
        candidate.force_derivatives.second,
        reference.yaw,
        reference.yaw_rate,
        reference.yaw_acceleration,
        candidate.desired_attitude)) {
    return false;
  }

  // Current body Z axis expressed in local NED.
  const Vector3 body_z{
    state.attitude(0, 2),
    state.attitude(1, 2),
    state.attitude(2, 2)
  };

  candidate.collective_thrust_n =
    -(candidate.translation.force * body_z);

  if (!std::isfinite(candidate.collective_thrust_n) ||
      candidate.collective_thrust_n <= 0.0f) {
    return false;
  }

  output = candidate;
  return true;
}

}  // namespace se3_control
