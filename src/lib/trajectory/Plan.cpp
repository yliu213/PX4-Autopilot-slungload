// Read and validate bounded trajectory files for onboard controllers.
// File parsing and plan storage occur outside the control update loop.

#include "Plan.hpp"

#include <cctype>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <pthread.h>

namespace trajectory_generator
{
namespace
{
constexpr char kSignature[] = "PX4_TRAJ_V2";
constexpr std::size_t kLineSize = 256;

pthread_mutex_t g_plan_mutex = PTHREAD_MUTEX_INITIALIZER;
Plan g_loaded_plan{};
bool g_has_plan{false};
unsigned g_revision{0};
// Handshake between the CLI and the SE3 work queue; guarded by g_plan_mutex.
bool g_se3_start_allowed{false};
bool g_start_pending{false};

uint32_t checksum_line(uint32_t current, const char *line)
{
  for (const unsigned char *p = reinterpret_cast<const unsigned char *>(line); *p; ++p) {
    current ^= *p;
    for (int bit = 0; bit < 8; ++bit) {
      current = (current >> 1) ^ ((current & 1U) ? 0xEDB88320U : 0U);
    }
  }
  return current;
}

bool blank_tail(const char *tail)
{
  for (; *tail; ++tail) {
    if (!std::isspace(static_cast<unsigned char>(*tail))) {
      return false;
    }
  }

  return true;
}

bool read_line(FILE *file, char *buffer, std::size_t capacity)
{
  if (!std::fgets(buffer, static_cast<int>(capacity), file)) {
    return false;
  }

  // An overlong line must never be interpreted as a truncated valid record.
  const std::size_t length = std::strlen(buffer);
  return length > 0 && (buffer[length - 1] == '\n' || std::feof(file));
}

bool parse_origin(const char *line, Reference &origin)
{
  int consumed = 0;
  const int count = std::sscanf(line, " %f %f %f %f %n",
                                &origin.position[0], &origin.position[1],
                                &origin.position[2], &origin.yaw, &consumed);
  return count == 4 && consumed > 0 && blank_tail(line + consumed) &&
         std::isfinite(origin.position[0]) &&
         std::isfinite(origin.position[1]) &&
         std::isfinite(origin.position[2]) && std::isfinite(origin.yaw);
}

bool parse_count(const char *line, std::size_t &count)
{
  unsigned value = 0;
  int consumed = 0;
  const int matched = std::sscanf(line, " %u %n", &value, &consumed);

  if (matched != 1 || consumed <= 0 || !blank_tail(line + consumed) ||
      value == 0 || value > kMaxSegments) {
    return false;
  }

  count = value;
  return true;
}

bool parse_segment(const char *line, Segment &segment)
{
  unsigned type = 0;
  int consumed = 0;
  const int matched = std::sscanf(line,
      " %u %f %f %f %f %f %f %f %f %f %f %n",
      &type, &segment.duration_s,
      &segment.offset[0], &segment.offset[1], &segment.offset[2],
      &segment.radius_m, &segment.height_m, &segment.turns,
      &segment.yaw_offset_rad, &segment.x_amplitude_m,
      &segment.y_amplitude_m, &consumed);

  if (matched != 11 || consumed <= 0 || !blank_tail(line + consumed) ||
      type > static_cast<unsigned>(Segment::Type::FigureEight)) {
    return false;
  }

  segment.type = static_cast<Segment::Type>(type);
  return true;
}
}  // namespace

bool load_plan_file(const char *path, Plan &plan, const char *&error)
{
  error = "invalid trajectory";

  if (!path || !*path) {
    error = "missing path";
    return false;
  }

  FILE *file = std::fopen(path, "r");

  if (!file) {
    error = "cannot open trajectory file";
    return false;
  }

  Plan candidate{};
  char line[kLineSize]{};
  bool ok = false;
  uint32_t crc = 0xFFFFFFFFU;

  do {
    if (!read_line(file, line, sizeof(line)) ||
        std::strncmp(line, kSignature, sizeof(kSignature) - 1) != 0 ||
        !blank_tail(line + sizeof(kSignature) - 1)) {
      error = "invalid plan version";
      break;
    }

    crc = checksum_line(crc, line);

    if (!read_line(file, line, sizeof(line)) || !parse_origin(line, candidate.origin)) {
      error = "invalid absolute NED origin";
      break;
    }

    crc = checksum_line(crc, line);

    if (!read_line(file, line, sizeof(line)) || !parse_count(line, candidate.sequence.count)) {
      error = "invalid segment count (1..32)";
      break;
    }

    crc = checksum_line(crc, line);
    bool segments_valid = true;

    for (std::size_t index = 0; index < candidate.sequence.count; ++index) {
      if (!read_line(file, line, sizeof(line)) ||
          !parse_segment(line, candidate.sequence.segments[index])) {
        segments_valid = false;
        break;
      }

      crc = checksum_line(crc, line);
      candidate.duration_s += candidate.sequence.segments[index].duration_s;
    }

    if (!segments_valid || !validate(candidate.sequence) ||
        !std::isfinite(candidate.duration_s)) {
      error = "invalid trajectory segment or duration";
      break;
    }

    // Verify the exact uploaded bytes, including any segment parameters.
    if (!read_line(file, line, sizeof(line))) {
      error = "missing trajectory checksum";
      break;
    }

    unsigned supplied_crc = 0;
    int consumed = 0;
    if (std::sscanf(line, " CRC32 %8x %n", &supplied_crc, &consumed) != 1 ||
        consumed <= 0 || !blank_tail(line + consumed) ||
        supplied_crc != (crc ^ 0xFFFFFFFFU)) {
      error = "trajectory checksum mismatch";
      break;
    }

    candidate.checksum = supplied_crc;

    // Disallow accidental appended commands/data after the fixed sequence.
    while (std::fgets(line, sizeof(line), file)) {
      if (!blank_tail(line)) {
        segments_valid = false;
        break;
      }
    }

    if (!segments_valid || std::ferror(file)) {
      error = "unexpected trailing content";
      break;
    }

    Reference first{};
    Reference last{};

    if (!sample(candidate.sequence, 0.0f, candidate.origin, first) ||
        !sample(candidate.sequence, candidate.duration_s + 1.0f, candidate.origin, last)) {
      error = "trajectory sampling failed";
      break;
    }

    const auto close = [](float a, float b) { return std::fabs(a - b) <= 0.01f; };
    if (!close(first.position[0], candidate.origin.position[0]) ||
        !close(first.position[1], candidate.origin.position[1]) ||
        !close(first.position[2], candidate.origin.position[2]) ||
        !close(last.position[0], candidate.origin.position[0]) ||
        !close(last.position[1], candidate.origin.position[1]) ||
        !close(last.position[2], candidate.origin.position[2]) ||
        std::fabs(std::atan2(std::sin(last.yaw - candidate.origin.yaw),
                             std::cos(last.yaw - candidate.origin.yaw))) > 0.01f) {
      error = "trajectory must finish at its starting pose";
      break;
    }

    plan = candidate;
    ok = true;
  } while (false);

  std::fclose(file);
  return ok;
}

bool store_plan(const Plan &plan)
{
  if (!validate(plan.sequence) || !std::isfinite(plan.duration_s)) {
    return false;
  }

  pthread_mutex_lock(&g_plan_mutex);
  if (g_start_pending) {
    pthread_mutex_unlock(&g_plan_mutex);
    return false;
  }
  g_loaded_plan = plan;
  g_has_plan = true;
  ++g_revision;
  pthread_mutex_unlock(&g_plan_mutex);
  return true;
}

bool snapshot_plan(Plan &plan, unsigned *revision)
{
  pthread_mutex_lock(&g_plan_mutex);
  const bool loaded = g_has_plan;

  if (loaded) {
    plan = g_loaded_plan;
    if (revision) { *revision = g_revision; }
  }

  pthread_mutex_unlock(&g_plan_mutex);
  return loaded;
}

void set_se3_start_allowed(bool allowed)
{
  pthread_mutex_lock(&g_plan_mutex);
  g_se3_start_allowed = allowed;

  if (!allowed) {
    g_start_pending = false;
  }

  pthread_mutex_unlock(&g_plan_mutex);
}

bool request_se3_start(const Plan &plan)
{
  if (!validate(plan.sequence) || !std::isfinite(plan.duration_s)) {
    return false;
  }

  pthread_mutex_lock(&g_plan_mutex);
  const bool accepted = g_se3_start_allowed && !g_start_pending;

  if (accepted) {
    // An unrelated 'trajectory select' cannot replace this pending request.
    g_loaded_plan = plan;
    g_has_plan = true;
    ++g_revision;
    g_start_pending = true;
  }

  pthread_mutex_unlock(&g_plan_mutex);
  return accepted;
}

bool consume_start_request(Plan &plan)
{
  pthread_mutex_lock(&g_plan_mutex);
  const bool accepted = g_se3_start_allowed && g_start_pending;

  if (accepted) {
    plan = g_loaded_plan;
    g_start_pending = false;
    g_se3_start_allowed = false;
  }

  pthread_mutex_unlock(&g_plan_mutex);
  return accepted;
}

}  // namespace trajectory_generator
