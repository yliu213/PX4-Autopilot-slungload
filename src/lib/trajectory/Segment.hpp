// Controller-independent trajectory segments and sequences.
//
// Segments describe motion relative to a reference pose in local NED.
// A sequence combines segments without dynamic memory allocation.
//
// The mathematical sampler determines how each segment is evaluated.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Reference.hpp"

namespace trajectory_generator
{

struct Segment
{
  enum class Type : uint8_t
  {
    Hold,
    Step,
    Line,
    Circle,
    Helix,
    Yaw,
    FigureEight
  };

  Type type{Type::Hold};
  float duration_s{0.0f};

  // Translation in local NED [m].
  Vector3 offset{};

  // Circular and helical trajectory parameters.
  float radius_m{0.0f};
  float height_m{0.0f};
  float turns{0.0f};

  // Heading and figure-eight parameters.
  float yaw_offset_rad{0.0f};
  float x_amplitude_m{0.0f};
  float y_amplitude_m{0.0f};
};

// Fixed-capacity sequence for deterministic onboard execution.
// Only the first count segments are active.
constexpr std::size_t kMaxSegments = 32;

struct Sequence
{
  std::array<Segment, kMaxSegments> segments{};
  std::size_t count{0};
};

}  // namespace trajectory_generator
