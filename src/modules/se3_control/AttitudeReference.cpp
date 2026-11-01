// Analytic desired-attitude dynamics for the geometric SE(3) controller.
//
// Desired body axes follow Lee et al. (2010).
//
//   b3_d = -A / ||A||
//   b2_d = normalize(b3_d x b1_c)
//   b1_d = b2_d x b3_d
//
// The desired orientation is R_d = [b1_d b2_d b3_d].
//
// Analytic basis derivatives produce the desired angular velocity and
// acceleration without finite-differencing measured vehicle states.

#include "AttitudeReference.hpp"

#include <cmath>

namespace se3_control
{
namespace
{

// Avoid poorly conditioned normalization in single precision.
constexpr float kDirectionTolerance = 1.0e-6f;

struct DirectionDynamics
{
  Vector3 value{};
  Vector3 first{};
  Vector3 second{};
};

struct BasisDynamics
{
  DirectionDynamics b1{};
  DirectionDynamics b2{};
  DirectionDynamics b3{};
};

// Differentiate a normalized vector n = v / ||v|| analytically.
//
// The first and second derivatives account for changes in the vector's
// magnitude. A nearly zero vector cannot define a reliable direction.
bool normalized_dynamics(
  const Vector3 & vector,
  const Vector3 & first,
  const Vector3 & second,
  DirectionDynamics & output)
{
  if (!vector.isAllFinite() ||
      !first.isAllFinite() ||
      !second.isAllFinite()) {
    return false;
  }

  const float magnitude = std::sqrt(vector * vector);

  if (!std::isfinite(magnitude) ||
      magnitude <= kDirectionTolerance) {
    return false;
  }

  const Vector3 direction = vector * (1.0f / magnitude);
  const float magnitude_first = direction * first;

  const Vector3 direction_first =
    (first - direction * magnitude_first) * (1.0f / magnitude);

  const float magnitude_second =
    (first * first + vector * second -
     magnitude_first * magnitude_first) / magnitude;

  const Vector3 direction_second =
    (second -
     direction_first * (2.0f * magnitude_first) -
     direction * magnitude_second) * (1.0f / magnitude);

  if (!direction.isAllFinite() ||
      !direction_first.isAllFinite() ||
      !direction_second.isAllFinite()) {
    return false;
  }

  output = {direction, direction_first, direction_second};
  return true;
}

// Construct the desired body axes and their first two time derivatives.
bool desired_basis(
  const Vector3 & force,
  const Vector3 & force_first,
  const Vector3 & force_second,
  float yaw,
  float yaw_rate,
  float yaw_acceleration,
  BasisDynamics & basis)
{
  DirectionDynamics b3{};

  if (!normalized_dynamics(
        -force, -force_first, -force_second, b3)) {
    return false;
  }

  const float cosine = std::cos(yaw);
  const float sine = std::sin(yaw);

  // Horizontal heading direction and its analytic derivatives.
  const Vector3 heading{cosine, sine, 0.0f};

  const Vector3 heading_first{
    -sine * yaw_rate,
    cosine * yaw_rate,
    0.0f
  };

  const Vector3 heading_second{
    -cosine * yaw_rate * yaw_rate - sine * yaw_acceleration,
    -sine * yaw_rate * yaw_rate + cosine * yaw_acceleration,
    0.0f
  };

  // The second desired body axis must be perpendicular to both the
  // desired thrust direction and the horizontal heading direction.
  const Vector3 b2_raw = b3.value.cross(heading);

  const Vector3 b2_raw_first =
    b3.first.cross(heading) +
    b3.value.cross(heading_first);

  const Vector3 b2_raw_second =
    b3.second.cross(heading) +
    b3.first.cross(heading_first) * 2.0f +
    b3.value.cross(heading_second);

  DirectionDynamics b2{};

  if (!normalized_dynamics(
        b2_raw, b2_raw_first, b2_raw_second, b2)) {
    return false;
  }

  // Complete the right-handed orthonormal body basis.
  const DirectionDynamics b1{
    b2.value.cross(b3.value),

    b2.first.cross(b3.value) +
    b2.value.cross(b3.first),

    b2.second.cross(b3.value) +
    b2.first.cross(b3.first) * 2.0f +
    b2.value.cross(b3.second)
  };

  if (!b1.value.isAllFinite() ||
      !b1.first.isAllFinite() ||
      !b1.second.isAllFinite()) {
    return false;
  }

  basis = {b1, b2, b3};
  return true;
}

// Recover body angular velocity from R_d^T Rdot_d = hat(Omega_d).
//
// The skew-symmetric components are averaged to suppress small numerical
// losses of orthogonality from floating-point calculations.
Vector3 angular_velocity(const BasisDynamics & basis)
{
  return {
    0.5f * (
      basis.b3.value * basis.b2.first -
      basis.b2.value * basis.b3.first
    ),
    0.5f * (
      basis.b1.value * basis.b3.first -
      basis.b3.value * basis.b1.first
    ),
    0.5f * (
      basis.b2.value * basis.b1.first -
      basis.b1.value * basis.b2.first
    )
  };
}

// Recover angular acceleration from the differentiated rotation basis.
// Quadratic Rdot terms are symmetric and vanish in the skew projection.
Vector3 angular_acceleration(const BasisDynamics & basis)
{
  return {
    0.5f * (
      basis.b3.value * basis.b2.second -
      basis.b2.value * basis.b3.second
    ),
    0.5f * (
      basis.b1.value * basis.b3.second -
      basis.b3.value * basis.b1.second
    ),
    0.5f * (
      basis.b2.value * basis.b1.second -
      basis.b1.value * basis.b2.second
    )
  };
}

}  // namespace

bool AttitudeReference::compute(
  const Vector3 & force,
  const Vector3 & force_derivative,
  const Vector3 & force_second_derivative,
  float yaw,
  float yaw_rate,
  float yaw_acceleration,
  DesiredAttitudeDynamics & output)
{
  if (!std::isfinite(yaw) ||
      !std::isfinite(yaw_rate) ||
      !std::isfinite(yaw_acceleration)) {
    return false;
  }

  BasisDynamics basis{};

  if (!desired_basis(
        force,
        force_derivative,
        force_second_derivative,
        yaw,
        yaw_rate,
        yaw_acceleration,
        basis)) {
    return false;
  }

  DesiredAttitudeDynamics candidate{};

  candidate.attitude.setCol(0, basis.b1.value);
  candidate.attitude.setCol(1, basis.b2.value);
  candidate.attitude.setCol(2, basis.b3.value);

  candidate.angular_velocity = angular_velocity(basis);
  candidate.angular_acceleration = angular_acceleration(basis);

  if (!candidate.attitude.isAllFinite() ||
      !candidate.angular_velocity.isAllFinite() ||
      !candidate.angular_acceleration.isAllFinite()) {
    return false;
  }

  output = candidate;
  return true;
}

}  // namespace se3_control
