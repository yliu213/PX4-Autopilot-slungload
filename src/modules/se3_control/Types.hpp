// Vehicle state and controller-specific data structures.
//
// Position, velocity, and acceleration use PX4 local NED.
// Attitude maps the body FRD frame to local NED.
// Angular velocity is expressed in the body FRD frame.

#pragma once

#include <matrix/matrix/math.hpp>

namespace se3_control
{

using Vector3 = matrix::Vector3f;
using Rotation = matrix::Dcmf;
using InertiaMatrix = matrix::Matrix3f;  // vehicle inertia matrix

struct State
{
  Vector3 position{};
  Vector3 velocity{};
  Rotation attitude{};
  Vector3 angular_velocity{};
};

struct TranslationGains
{
  float mass{0.0f};       // Vehicle mass [kg].
  float position{0.0f};   // Position gain [N/m].
  float velocity{0.0f};   // Velocity gain [N s/m].
};

struct TranslationOutput
{
  // Kinematic acceleration command in local NED [m/s^2].
  Vector3 acceleration{};

  // Desired force including gravity compensation, in NED [N].
  // This is not a normalized PX4 thrust setpoint.
  Vector3 force{};
};

// Desired attitude and its analytic derivatives.
//
// The rotation maps desired body FRD to local NED. Angular velocity and
// acceleration are expressed in the desired body FRD frame.
struct DesiredAttitudeDynamics
{
  Rotation attitude{};
  Vector3 angular_velocity{};       // [rad/s].
  Vector3 angular_acceleration{};   // [rad/s^2].
};

// PX4-normalized geometric attitude gains.
// The feedforward gain multiplies angular acceleration [rad/s^2].
struct NormalizedAttitudeGains
{
  Vector3 attitude{};
  Vector3 angular_velocity{};
  Vector3 angular_acceleration{};
};

// Physical Lee geometric controller gains.
// attitude: [N m]
// angular_velocity: [N m s]
struct PhysicalAttitudeGains
{
    Vector3 attitude{};
    Vector3 angular_velocity{};
};

struct NormalizedTorqueOutput
{
  Vector3 attitude_feedback{};
  Vector3 angular_velocity_feedback{};
  Vector3 acceleration_feedforward{};
  Vector3 torque{};
};

// First and second time derivatives of the desired force in NED.
//
// These derivatives drive the analytic desired-attitude dynamics.
// Units are N/s and N/s^2, respectively.
struct ForceDerivatives
{
  Vector3 first{};
  Vector3 second{};
};

// Common geometric reference calculated before selecting a torque law.
//
// The onboard rate and normalized torque controllers share this attitude.
// Collective thrust is a physical magnitude [N], not a PX4 command.
struct GeometricOutput
{
  TranslationOutput translation{};
  ForceDerivatives force_derivatives{};
  DesiredAttitudeDynamics desired_attitude{};
  float collective_thrust_n{0.0f};
};

}  // namespace se3_control
