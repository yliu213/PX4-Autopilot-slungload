// Analytic trajectory sampling from fixed-capacity segment sequences.
//
// Ninth-order time scaling provides derivatives through snap and smooth
// boundaries between stationary and continuously interpolated segments.

#include "Sampler.hpp"

#include <algorithm>
#include <cmath>

namespace trajectory_generator
{
namespace
{

constexpr float kPi = 3.14159265358979323846f;

struct TimeScale
{
  float s;
  float d1;
  float d2;
  float d3;
  float d4;
};

struct ScalarDynamics
{
  float value;
  float d1;
  float d2;
  float d3;
  float d4;
};

bool finite(const Vector3 & vector)
{
  return std::isfinite(vector[0]) &&
         std::isfinite(vector[1]) &&
         std::isfinite(vector[2]);
}

bool finite(const Reference & reference)
{
  return finite(reference.position) &&
         finite(reference.velocity) &&
         finite(reference.acceleration) &&
         finite(reference.jerk) &&
         finite(reference.snap) &&
         std::isfinite(reference.yaw) &&
         std::isfinite(reference.yaw_rate) &&
         std::isfinite(reference.yaw_acceleration);
}

Vector3 add(const Vector3 & first, const Vector3 & second)
{
  return {
    first[0] + second[0],
    first[1] + second[1],
    first[2] + second[2]
  };
}

Vector3 scale(const Vector3 & vector, float factor)
{
  return {
    vector[0] * factor,
    vector[1] * factor,
    vector[2] * factor
  };
}

Reference stationary(const Vector3 & position, float yaw)
{
  Reference reference{};
  reference.position = position;
  reference.yaw = yaw;
  return reference;
}

// Ninth-order C4 interpolation. Derivatives through snap vanish at both
// endpoints, allowing smooth segments to join stationary references.
TimeScale smooth_time_scale(float time_s, float duration_s)
{
  const float t = std::max(0.0f, std::min(time_s, duration_s));
  const float u = t / duration_s;

  // P(1-u) = 1-P(u). Work only on 0 <= w <= 0.5 so its
  // polynomial does not subtract large, nearly equal numbers.
  const bool upper_half = u > 0.5f;
  const float w = upper_half ? 1.0f - u : u;
  const float w2 = w * w;
  const float w3 = w2 * w;
  const float w4 = w3 * w;
  const float one_minus_w = 1.0f - w;
  const float v2 = one_minus_w * one_minus_w;
  const float v3 = v2 * one_minus_w;
  const float v4 = v3 * one_minus_w;

  // P(w) = 126w^5 - 420w^6 + 540w^7 - 315w^8 + 70w^9.
  const float p = w4 * w *
                  (126.0f + w * (-420.0f + w *
                   (540.0f + w * (-315.0f + 70.0f * w))));

  // Exact derivatives with respect to dimensionless u.
  const float d1 = 630.0f * w4 * v4;
  const float d2 = 2520.0f * w3 * v3 * (1.0f - 2.0f * w);
  const float d3 = 2520.0f * w2 * v2 * (3.0f - 14.0f * w + 14.0f * w2);
  const float d4 = 15120.0f * w * one_minus_w * (1.0f - 2.0f * w) *
                   (1.0f - 7.0f * w + 7.0f * w2);

  const float inv_t = 1.0f / duration_s;
  const float inv_t2 = inv_t * inv_t;
  const float inv_t3 = inv_t2 * inv_t;
  const float inv_t4 = inv_t3 * inv_t;

  // P' and P''' are symmetric; P'' and P'''' are antisymmetric.
  return {
    upper_half ? 1.0f - p : p,
    d1 * inv_t,
    (upper_half ? -d2 : d2) * inv_t2,
    d3 * inv_t3,
    (upper_half ? -d4 : d4) * inv_t4
  };
}

// Analytical derivatives of amplitude*sin(phase(t)) through fourth order.
ScalarDynamics sinusoid(
  float amplitude,
  float phase,
  float p1,
  float p2,
  float p3,
  float p4)
{
  const float s = std::sin(phase);
  const float c = std::cos(phase);

  return {
    amplitude * s,
    amplitude * c * p1,
    amplitude * (-s * p1 * p1 + c * p2),
    amplitude * (-c * p1 * p1 * p1 - 3.0f * s * p1 * p2 + c * p3),
    amplitude * (
      s * p1 * p1 * p1 * p1 -
      6.0f * c * p1 * p1 * p2 -
      3.0f * s * p2 * p2 -
      4.0f * s * p1 * p3 +
      c * p4
    )
  };
}

Reference sample_line(
  const Segment & segment,
  const Reference & origin,
  const TimeScale & q)
{
  Reference reference{};

  reference.position = add(origin.position, scale(segment.offset, q.s));
  reference.velocity = scale(segment.offset, q.d1);
  reference.acceleration = scale(segment.offset, q.d2);
  reference.jerk = scale(segment.offset, q.d3);
  reference.snap = scale(segment.offset, q.d4);
  reference.yaw = origin.yaw;

  return reference;
}

Reference sample_yaw(
  const Segment & segment,
  const Reference & origin,
  const TimeScale & q)
{
  Reference reference =
    stationary(origin.position, origin.yaw + segment.yaw_offset_rad * q.s);

  reference.yaw_rate = segment.yaw_offset_rad * q.d1;
  reference.yaw_acceleration = segment.yaw_offset_rad * q.d2;

  return reference;
}

Reference sample_figure_eight(
  const Segment & segment,
  const Reference & origin,
  const TimeScale & q)
{
  const float angle = 2.0f * kPi * segment.turns * q.s;
  const float a1 = 2.0f * kPi * segment.turns * q.d1;
  const float a2 = 2.0f * kPi * segment.turns * q.d2;
  const float a3 = 2.0f * kPi * segment.turns * q.d3;
  const float a4 = 2.0f * kPi * segment.turns * q.d4;

  const ScalarDynamics x =
    sinusoid(segment.x_amplitude_m, angle, a1, a2, a3, a4);

  // Twice the x phase produces the planar 1:2 Lissajous curve.
  const ScalarDynamics y =
    sinusoid(
      segment.y_amplitude_m,
      2.0f * angle,
      2.0f * a1,
      2.0f * a2,
      2.0f * a3,
      2.0f * a4
    );

  Reference reference{};

  reference.position = {
    origin.position[0] + x.value,
    origin.position[1] + y.value,
    origin.position[2]
  };

  reference.velocity = {x.d1, y.d1, 0.0f};
  reference.acceleration = {x.d2, y.d2, 0.0f};
  reference.jerk = {x.d3, y.d3, 0.0f};
  reference.snap = {x.d4, y.d4, 0.0f};
  reference.yaw = origin.yaw;

  return reference;
}

Reference sample_circle(
  const Segment & segment,
  const Reference & origin,
  const TimeScale & q)
{
  const float angle = 2.0f * kPi * segment.turns * q.s;

  const float w1 = 2.0f * kPi * segment.turns * q.d1;
  const float w2 = 2.0f * kPi * segment.turns * q.d2;
  const float w3 = 2.0f * kPi * segment.turns * q.d3;
  const float w4 = 2.0f * kPi * segment.turns * q.d4;

  const float c = std::cos(angle);
  const float s = std::sin(angle);

  const float r = segment.radius_m;
  const float h =
    segment.type == Segment::Type::Helix ? segment.height_m : 0.0f;

  Reference reference{};

  // The segment starts at origin.position. Its circle center is one radius
  // toward negative local-NED x, so entry has no position discontinuity.
  reference.position = {
    origin.position[0] + r * (c - 1.0f),
    origin.position[1] + r * s,
    origin.position[2] + h * q.s
  };

  reference.velocity = {
    -r * s * w1,
    r * c * w1,
    h * q.d1
  };

  reference.acceleration = {
    -r * (c * w1 * w1 + s * w2),
    r * (-s * w1 * w1 + c * w2),
    h * q.d2
  };

  reference.jerk = {
    r * (s * w1 * w1 * w1 - 3.0f * c * w1 * w2 - s * w3),
    r * (-c * w1 * w1 * w1 - 3.0f * s * w1 * w2 + c * w3),
    h * q.d3
  };

  reference.snap = {
    r * (
      6.0f * s * w1 * w1 * w2 -
      s * w4 +
      c * w1 * w1 * w1 * w1 -
      4.0f * c * w1 * w3 -
      3.0f * c * w2 * w2
    ),
    r * (
      s * w1 * w1 * w1 * w1 -
      4.0f * s * w1 * w3 -
      3.0f * s * w2 * w2 -
      6.0f * c * w1 * w1 * w2 +
      c * w4
    ),
    h * q.d4
  };

  reference.yaw = origin.yaw;

  return reference;
}

// Select a primitive while keeping sequence logic independent of its shape.
Reference sample_segment(
  const Segment & segment,
  float time_s,
  const Reference & origin)
{
  if (segment.type == Segment::Type::Hold) {
    return stationary(origin.position, origin.yaw);
  }

  if (segment.type == Segment::Type::Step) {
    // A step is instantaneous, including at its segment start.
    return stationary(add(origin.position, segment.offset), origin.yaw);
  }

  const TimeScale q = smooth_time_scale(time_s, segment.duration_s);

  switch (segment.type) {
  case Segment::Type::Line:
    return sample_line(segment, origin, q);

  case Segment::Type::Circle:
  case Segment::Type::Helix:
    return sample_circle(segment, origin, q);

  case Segment::Type::Yaw:
    return sample_yaw(segment, origin, q);

  case Segment::Type::FigureEight:
    return sample_figure_eight(segment, origin, q);

  default:
    return Reference{};
  }
}

}  // namespace

// Reject invalid sequences before sampling. This also protects the bounded
// array against an invalid count and prevents non-finite trajectory inputs.
bool validate(const Sequence & sequence)
{
  if (sequence.count == 0 || sequence.count > kMaxSegments) {
    return false;
  }

  float total_duration_s = 0.0f;

  for (std::size_t index = 0; index < sequence.count; ++index) {
    const Segment & segment = sequence.segments[index];

    if (!std::isfinite(segment.duration_s) || segment.duration_s < 0.01f ||
        !finite(segment.offset) ||
        !std::isfinite(segment.radius_m) ||
        !std::isfinite(segment.height_m) ||
        !std::isfinite(segment.turns) ||
        !std::isfinite(segment.yaw_offset_rad) ||
        !std::isfinite(segment.x_amplitude_m) ||
        !std::isfinite(segment.y_amplitude_m)) {
      return false;
    }

    if ((segment.type == Segment::Type::Circle ||
         segment.type == Segment::Type::Helix) &&
        segment.radius_m <= 0.0f) {
      return false;
    }

    if (segment.type == Segment::Type::FigureEight &&
        (segment.x_amplitude_m <= 0.0f ||
         segment.y_amplitude_m <= 0.0f ||
         segment.turns <= 0.0f)) {
      return false;
    }

    if (segment.type > Segment::Type::FigureEight) {
      return false;
    }

    total_duration_s += segment.duration_s;

    if (!std::isfinite(total_duration_s)) {
      return false;
    }
  }

  return true;
}

// Evaluate the sequence using configured NED reference coordinates.
// The previous segment's ending position and yaw anchor the next segment.
// At an exact boundary, the ending segment owns the returned sample.
bool sample(
  const Sequence & sequence,
  float time_s,
  const Reference & origin,
  Reference & reference)
{
  if (!validate(sequence) ||
      !std::isfinite(time_s) ||
      !finite(origin.position) ||
      !std::isfinite(origin.yaw)) {
    return false;
  }

  Reference segment_origin = stationary(origin.position, origin.yaw);
  float remaining_s = std::max(0.0f, time_s);

  for (std::size_t index = 0; index < sequence.count; ++index) {
    const Segment & segment = sequence.segments[index];

    if (remaining_s <= segment.duration_s) {
      const Reference candidate =
        sample_segment(segment, remaining_s, segment_origin);

      if (!finite(candidate)) {
        return false;
      }

      reference = candidate;
      return true;
    }

    segment_origin =
      sample_segment(segment, segment.duration_s, segment_origin);

    if (!finite(segment_origin)) {
      return false;
    }

    remaining_s -= segment.duration_s;
  }

  // Once every segment has completed, hold the final pose and heading.
  reference = stationary(segment_origin.position, segment_origin.yaw);

  return true;
}

}  // namespace trajectory_generator
