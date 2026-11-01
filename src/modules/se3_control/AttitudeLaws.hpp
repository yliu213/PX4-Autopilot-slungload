// Geometric attitude control laws.
//
// Both control laws share the same geometric SO(3) tracking errors.

#pragma once

#include "Types.hpp"

namespace se3_control
{

class AttitudeLaws
{
public:
  // Geometric attitude-to-body-rate handoff [rad/s].
  // Omega_sp = R^T R_d Omega_d - K_R * e_R.
  // mc_rate_control owns normalized torque production.
  static bool body_rate(
    const State & state,
    const DesiredAttitudeDynamics & desired,
    const Vector3 & attitude_gain,
    Vector3 & rate_setpoint);

  // Lee geometric physical moment controller.
  // Output is physical body moment [N m].
  static bool physical(
    const State & state,
    const DesiredAttitudeDynamics & desired,
    const InertiaMatrix & inertia,
    const PhysicalAttitudeGains & gains,
    Vector3 & moment_nm);

  // Direct PX4-normalized body torque command.
  static bool normalized(
    const State & state,
    const DesiredAttitudeDynamics & desired,
    const NormalizedAttitudeGains & gains,
    NormalizedTorqueOutput & output);

};

}  // namespace se3_control
