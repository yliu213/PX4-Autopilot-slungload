// Geometric SO(3) body-rate and normalized torque control.
//
// Geometric attitude error (Lee et al., 2010):
//
//   e_R = 0.5 (R_d^T R - R^T R_d)^vee
//
// Angular velocity tracking error:
//
//   e_Omega = Omega - R^T R_d Omega_d
//
// Both errors are expressed in the current body FRD frame.

#include "AttitudeLaws.hpp"

#include <cmath>

namespace se3_control
{
namespace
{

struct AttitudeErrors
{
  Vector3 attitude{};
  Vector3 angular_velocity{};
  Vector3 desired_rate_current{};
  Vector3 acceleration_feedforward{};
};

bool nonnegative(const Vector3 & values)
{
  return values.isAllFinite() &&
         values(0) >= 0.0f &&
         values(1) >= 0.0f &&
         values(2) >= 0.0f;
}

Vector3 component_product(
  const Vector3 & first,
  const Vector3 & second)
{
  return {
    first(0) * second(0),
    first(1) * second(1),
    first(2) * second(2)
  };
}

// Calculate the common geometric errors and desired acceleration term.
bool compute_errors(
  const State & state,
  const DesiredAttitudeDynamics & desired,
  AttitudeErrors & output)
{
  if (!state.attitude.isAllFinite() ||
      !state.angular_velocity.isAllFinite() ||
      !desired.attitude.isAllFinite() ||
      !desired.angular_velocity.isAllFinite() ||
      !desired.angular_acceleration.isAllFinite()) {
    return false;
  }

  const auto relative =
    desired.attitude.transpose() * state.attitude;

  // Skew-symmetric part of R_d^T R.
  const Vector3 attitude_error{
    0.5f * (relative(2, 1) - relative(1, 2)),
    0.5f * (relative(0, 2) - relative(2, 0)),
    0.5f * (relative(1, 0) - relative(0, 1))
  };

  // Map desired-body angular velocity into the current body frame.
  const Vector3 desired_rate_current =
    state.attitude.transpose() *
    (desired.attitude * desired.angular_velocity);

  const Vector3 angular_velocity_error =
    state.angular_velocity - desired_rate_current;

  // Time derivative of R^T R_d Omega_d, in the current body frame.
  //
  // alpha_ff = -Omega x (R^T R_d Omega_d)
  //            + R^T R_d dot(Omega_d)
  const Vector3 acceleration_feedforward =
    -state.angular_velocity.cross(desired_rate_current) +
    state.attitude.transpose() *
    (desired.attitude * desired.angular_acceleration);

  if (!attitude_error.isAllFinite() ||
      !angular_velocity_error.isAllFinite() ||
      !acceleration_feedforward.isAllFinite()) {
    return false;
  }

  output = {
    attitude_error,
    angular_velocity_error,
    desired_rate_current,
    acceleration_feedforward
  };

  return true;
}

}  // namespace

bool AttitudeLaws::body_rate(
  const State & state,
  const DesiredAttitudeDynamics & desired,
  const Vector3 & attitude_gain,
  Vector3 & rate_setpoint)
{
  if (!nonnegative(attitude_gain)) {
    return false;
  }

  AttitudeErrors errors{};

  if (!compute_errors(state, desired, errors)) {
    return false;
  }

  // Same law as the validated ROS 2 SE(3) attitude_rate handoff.
  // Omega_sp = R^T R_d Omega_d - K_R e_R.
  const Vector3 candidate = errors.desired_rate_current -
                            component_product(attitude_gain, errors.attitude);

  if (!candidate.isAllFinite()) {
    return false;
  }

  rate_setpoint = candidate;
  return true;
}

bool AttitudeLaws::normalized(
  const State & state,
  const DesiredAttitudeDynamics & desired,
  const NormalizedAttitudeGains & gains,
  NormalizedTorqueOutput & output)
{
  if (!nonnegative(gains.attitude) ||
      !nonnegative(gains.angular_velocity) ||
      !nonnegative(gains.angular_acceleration)) {
    return false;
  }

  AttitudeErrors errors{};

  if (!compute_errors(state, desired, errors)) {
    return false;
  }

  // Normalized geometric torque:
  //
  // tau_n = -K_R e_R - K_Omega e_Omega + K_alpha alpha_ff
  //
  // No physical inertia or N*m conversion occurs in this law.
  NormalizedTorqueOutput candidate{};

  candidate.attitude_feedback =
    -component_product(gains.attitude, errors.attitude);

  candidate.angular_velocity_feedback =
    -component_product(
      gains.angular_velocity,
      errors.angular_velocity);

  candidate.acceleration_feedforward =
    component_product(
      gains.angular_acceleration,
      errors.acceleration_feedforward);

  candidate.torque =
    candidate.attitude_feedback +
    candidate.angular_velocity_feedback +
    candidate.acceleration_feedforward;

  if (!candidate.torque.isAllFinite()) {
    return false;
  }

  output = candidate;
  return true;
}

bool AttitudeLaws::physical(
    const State & state,
    const DesiredAttitudeDynamics & desired,
    const InertiaMatrix & inertia,
    const PhysicalAttitudeGains & gains,
    Vector3 & moment_nm)
{
    // Validate physical gains and inertia.
    if (!nonnegative(gains.attitude) ||
        !nonnegative(gains.angular_velocity) ||
        !inertia.isAllFinite() ||
        inertia(0, 0) <= 0.f ||
        inertia(1, 1) <= 0.f ||
        inertia(2, 2) <= 0.f) {
        return false;
    }

    // Reuse the existing SO(3) error calculation.
    AttitudeErrors errors{};

    if (!compute_errors(state, desired, errors)) {
        return false;
    }

    // Physical angular momentum: J * Omega
    const Vector3 angular_momentum =
        inertia * state.angular_velocity;

    // Lee geometric moment equation:
    //
    // M = -K_R * e_R - K_Omega * e_Omega
    //     + Omega x (J * Omega) + J * alpha_ff
    //
    // Output units: [N m]

    const Vector3 moment =
        -component_product(gains.attitude, errors.attitude)
        -component_product(gains.angular_velocity,
                           errors.angular_velocity)
        +state.angular_velocity.cross(angular_momentum)
        +inertia * errors.acceleration_feedforward;

    if (!moment.isAllFinite()) {
        return false;
    }

    moment_nm = moment;
    return true;
}

}  // namespace se3_control
