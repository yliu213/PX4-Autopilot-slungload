// Analytic desired-attitude reference for geometric control.
//
// The desired thrust direction and commanded heading determine the
// desired FRD-to-NED orientation. Their analytic derivatives provide
// angular velocity and angular acceleration feedforward.

#pragma once

#include "Types.hpp"

namespace se3_control
{

class AttitudeReference
{
public:
  // Forces are expressed in NED [N], with derivatives [N/s] and [N/s^2].
  // Yaw and its derivatives use rad, rad/s, and rad/s^2.
  //
  // Returns false for non-finite inputs or degenerate attitude geometry.
  // On failure, output remains unchanged.
  static bool compute(
    const Vector3 & force,
    const Vector3 & force_derivative,
    const Vector3 & force_second_derivative,
    float yaw,
    float yaw_rate,
    float yaw_acceleration,
    DesiredAttitudeDynamics & output);
};

}  // namespace se3_control
