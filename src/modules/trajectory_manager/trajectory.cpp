// Shell access to the shared, validated, persistent trajectory catalog.
// Selection only stages a plan; start also explicitly requests SE3 execution.

#include <trajectory/Plan.hpp>

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <cstdio>
#include <cstring>
#include <dirent.h>

namespace
{
#if defined(__PX4_POSIX)
constexpr char kCatalog[] = "fs/microsd/trajectories";
#else
constexpr char kCatalog[] = "/fs/microsd/trajectories";
#endif

bool valid_name(const char *name)
{
    if (!name || !*name || std::strlen(name) > 31) { return false; }
    for (const char *p = name; *p; ++p) {
        const bool alpha = (*p >= 'a' && *p <= 'z') || (*p >= 'A' && *p <= 'Z');
        const bool digit = *p >= '0' && *p <= '9';
        if (!alpha && !digit && *p != '_' && *p != '-') { return false; }
    }
    return true;
}

bool ends_with_traj(const char *s)
{
    const std::size_t n = std::strlen(s);
    return n > 5 && std::strcmp(s + n - 5, ".traj") == 0;
}
}

extern "C" __EXPORT int trajectory_main(int argc, char *argv[])
{
    if (argc == 2 && std::strcmp(argv[1], "list") == 0) {
        DIR *folder = opendir(kCatalog);
        if (!folder) { PX4_ERR("cannot open catalog: %s", kCatalog); return PX4_ERROR; }
        PX4_INFO("onboard catalog: %s", kCatalog);
        struct dirent *entry;
        while ((entry = readdir(folder)) != nullptr) {
            if (ends_with_traj(entry->d_name)) { PX4_INFO("  %s", entry->d_name); }
        }
        closedir(folder);
        return PX4_OK;
    }

    if (argc == 3 && (std::strcmp(argv[1], "select") == 0 ||
                      std::strcmp(argv[1], "start") == 0)) {
        if (!valid_name(argv[2])) {
            PX4_ERR("invalid trajectory name");
            return PX4_ERROR;
        }
        char path[256];
        const int length = std::snprintf(path, sizeof(path), "%s/%s.traj", kCatalog, argv[2]);
        if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(path)) {
            PX4_ERR("trajectory path too long");
            return PX4_ERROR;
        }
        trajectory_generator::Plan plan{};
        const char *error = nullptr;
        if (!trajectory_generator::load_plan_file(path, plan, error)) {
            PX4_ERR("invalid trajectory %s: %s", argv[2], error);
            return PX4_ERROR;
        }
        std::strncpy(plan.name, argv[2], sizeof(plan.name) - 1);
        const bool start = std::strcmp(argv[1], "start") == 0;

        if (start) {
            // This request is accepted only while the registered SE3 mode is
            // active and holding. No start-pose/yaw alignment is performed.
            if (!trajectory_generator::request_se3_start(plan)) {
                PX4_ERR("SE3 must be active and holding; start not requested");
                return PX4_ERROR;
            }
            PX4_INFO("start requested: %s (%u segments, %.2f s)", plan.name,
                     static_cast<unsigned>(plan.sequence.count), static_cast<double>(plan.duration_s));
        } else {
            if (!trajectory_generator::store_plan(plan)) {
                PX4_ERR("trajectory storage rejected plan");
                return PX4_ERROR;
            }
            PX4_INFO("selected %s: %u segments, %.2f s", plan.name,
                     static_cast<unsigned>(plan.sequence.count), static_cast<double>(plan.duration_s));
        }
        return PX4_OK;
    }

    if (argc == 2 && std::strcmp(argv[1], "status") == 0) {
        trajectory_generator::Plan plan{};
        if (!trajectory_generator::snapshot_plan(plan)) {
            PX4_INFO("no selected trajectory");
        } else {
            PX4_INFO("selected %s: %u segments, %.2f s", plan.name,
                     static_cast<unsigned>(plan.sequence.count), static_cast<double>(plan.duration_s));
            PX4_INFO("start NED [%.2f, %.2f, %.2f] yaw %.2f",
                     static_cast<double>(plan.origin.position[0]),
                     static_cast<double>(plan.origin.position[1]),
                     static_cast<double>(plan.origin.position[2]),
                     static_cast<double>(plan.origin.yaw));
        }
        return PX4_OK;
    }

    PX4_INFO("usage: trajectory list | select <name> | start <name> | status");
    return PX4_ERROR;
}
