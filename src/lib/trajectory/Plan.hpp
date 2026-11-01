// Shared, fixed-capacity trajectory plans for PX4 controllers.
//
// A selected plan contains absolute PX4 local NED start position and yaw.
// The SE3 controller consumes explicit start requests from the trajectory CLI.

#pragma once

#include <cstddef>
#include <cstdint>

#include "Sampler.hpp"

namespace trajectory_generator
{

struct Plan
{
  char name[32]{}; // Assigned by the onboard selection command.
  Reference origin{};
  Sequence sequence{};
  float duration_s{0.0f};
  uint32_t checksum{0}; // CRC32 of the loaded trajectory file.
};

// Text format version 2, produced by tools/trajectory_lib/compiler.py:
// PX4_TRAJ_V2
// x y z yaw
// segment_count
// type duration offset_x offset_y offset_z radius height turns yaw_offset x_amp y_amp
// ... repeated segment_count times. Unused fields are zero.
// CRC32 <8 hex digits> over every preceding line, including newlines.
// A failed load preserves the previous plan.
bool load_plan_file(const char *path, Plan &plan, const char *&error);

// Thread-safe, bounded storage shared by the trajectory CLI and any onboard
// controller linked to trajectory_generator. Snapshot uses a short mutex hold;
// parsing and sampling never occur while the store is locked.
bool store_plan(const Plan &plan);
bool snapshot_plan(Plan &plan, unsigned *revision = nullptr);

// A flight-mode-independent shell command cannot alter the active controller.
// SE3 sets this permission while selected and holding (never while executing).
// Clearing permission also cancels any pending start request.
void set_se3_start_allowed(bool allowed);

// Atomically install a validated plan and ask SE3 to execute it. Reject if
// SE3 is not holding or a previous start has not yet been consumed.
bool request_se3_start(const Plan &plan);

// Called only by the SE3 work queue. Consumes the requested plan once, so
// no previously selected plan can start automatically on mode entry.
bool consume_start_request(Plan &plan);

}  // namespace trajectory_generator
