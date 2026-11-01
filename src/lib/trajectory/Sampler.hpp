// Analytic trajectory sampling interface.
//
// Evaluates a bounded segment sequence at elapsed trajectory time.
// Invalid inputs are reported by the return value.

#pragma once

#include "Reference.hpp"
#include "Segment.hpp"

namespace trajectory_generator
{

// Validate all active segments before using a sequence.
bool validate(const Sequence & sequence);

// Evaluate the complete sequence at the requested elapsed time.
//
// The origin is the configured reference pose, not automatically the
// measured vehicle position. This preserves absolute NED trajectories.
//
// On failure, reference is left unchanged.
// After completion, the final position and yaw are held.
bool sample(
  const Sequence & sequence,
  float time_s,
  const Reference & origin,
  Reference & reference);

}  // namespace trajectory_generator
