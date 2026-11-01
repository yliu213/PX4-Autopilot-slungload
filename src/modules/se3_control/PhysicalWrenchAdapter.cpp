#include "PhysicalWrenchAdapter.hpp"

#include <parameters/param.h>

#include <algorithm>
#include <cmath>

namespace se3_control
{
namespace
{
constexpr int N = 4;
// F450 model.sdf: T = 1.2e-5 * (150 + 850*u)^2, u in [0,1].
constexpr float kTmin = 0.27f;
constexpr float kTmax = 12.0f;
constexpr float kC1 = 3.06f;
constexpr float kC2 = 8.67f;
inline float limit(float x, float lo, float hi) { return std::min(std::max(x, lo), hi); }

// 4x4 inverse, computed only when the PX4 configuration changes.
bool inverse4(const float a[N][N], float inverse[N][N])
{
    double aug[N][2 * N]{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) { aug[i][j] = a[i][j]; }
        aug[i][N + i] = 1.0;
    }
    for (int c = 0; c < N; ++c) {
        int pivot = c;
        for (int r = c + 1; r < N; ++r) {
            if (std::fabs(aug[r][c]) > std::fabs(aug[pivot][c])) { pivot = r; }
        }
        if (!std::isfinite(aug[pivot][c]) || std::fabs(aug[pivot][c]) < 1e-10) { return false; }
        for (int j = 0; j < 2 * N; ++j) { std::swap(aug[pivot][j], aug[c][j]); }
        const double d = aug[c][c];
        for (int j = 0; j < 2 * N; ++j) { aug[c][j] /= d; }
        for (int r = 0; r < N; ++r) {
            if (r == c) { continue; }
            const double f = aug[r][c];
            for (int j = 0; j < 2 * N; ++j) { aug[r][j] -= f * aug[c][j]; }
        }
    }
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) { inverse[i][j] = static_cast<float>(aug[i][N + j]); }
    }
    return true;
}

bool get_param(const char * name, float & value)
{
    const param_t handle = param_find(name);
    return handle != PARAM_INVALID && param_get(handle, &value) == 0 && std::isfinite(value);
}
} // namespace

bool PhysicalWrenchAdapter::load_configuration(Configuration & config)
{
    // Physical rotor positions, expressed in FRD, from F450 model.sdf.
    constexpr float x[N] = { 0.16263456f, -0.16263456f, 0.16263456f, -0.16263456f };
    constexpr float y[N] = { 0.16263456f, -0.16263456f, -0.16263456f, 0.16263456f };
    constexpr float km_phys[N] = { 0.0137f, 0.0137f, -0.0137f, -0.0137f };
    float physical[N][N]{};
    for (int i = 0; i < N; ++i) {
        physical[0][i] = -y[i];
        physical[1][i] = x[i];
        physical[2][i] = km_phys[i];
        physical[3][i] = 1.f;
    }
    if (!inverse4(physical, config.physical_inverse)) { return false; }

    // Read the ACTIVE PX4 allocator parameters, not saved YAML/airframe guesses.
    constexpr const char * names[N][4] = {
        {"CA_ROTOR0_PX", "CA_ROTOR0_PY", "CA_ROTOR0_CT", "CA_ROTOR0_KM"},
        {"CA_ROTOR1_PX", "CA_ROTOR1_PY", "CA_ROTOR1_CT", "CA_ROTOR1_KM"},
        {"CA_ROTOR2_PX", "CA_ROTOR2_PY", "CA_ROTOR2_CT", "CA_ROTOR2_KM"},
        {"CA_ROTOR3_PX", "CA_ROTOR3_PY", "CA_ROTOR3_CT", "CA_ROTOR3_KM"},
    };
    for (int i = 0; i < N; ++i) {
        float px{}, py{}, ct{}, km{};
        if (!get_param(names[i][0], px) || !get_param(names[i][1], py) ||
            !get_param(names[i][2], ct) || !get_param(names[i][3], km) || ct <= 0.f ||
            (i < 2 ? km <= 0.f : km >= 0.f)) { return false; }
        config.allocator[0][i] = -ct * py;
        config.allocator[1][i] = ct * px;
        config.allocator[2][i] = ct * km;
        config.allocator[3][i] = -ct;
    }
    float mix[N][N]{};
    if (!inverse4(config.allocator, mix)) { return false; }

    // PX4 ControlAllocationPseudoInverse normalization, F450 quad.
    float roll_sq = 0.f, pitch_sq = 0.f;
    int nr = 0, np = 0;
    for (int i = 0; i < N; ++i) {
        roll_sq += mix[i][0] * mix[i][0];
        pitch_sq += mix[i][1] * mix[i][1];
        if (std::fabs(mix[i][0]) > 1e-3f) { ++nr; }
        if (std::fabs(mix[i][1]) > 1e-3f) { ++np; }
    }
    if (nr == 0 || np == 0) { return false; }
    config.scale[0] = config.scale[1] =
        std::max(std::sqrt(roll_sq / (nr / 2.f)),
                 std::sqrt(pitch_sq / (np / 2.f)));
    config.scale[2] = mix[0][2];
    config.scale[3] = 0.f;
    for (int i = 0; i < N; ++i) {
        config.scale[2] = std::max(config.scale[2], mix[i][2]);
        config.scale[3] += std::fabs(mix[i][3]) / N;
    }
    for (float s : config.scale) {
        if (!std::isfinite(s) || s <= 0.f) { return false; }
    }
    return true;
}

bool PhysicalWrenchAdapter::adapt(const Configuration & config,
                                  float collective_n,
                                  const Vector3 & moment_nm,
                                  Output & output)
{
    if (!std::isfinite(collective_n) || collective_n < 0.f ||
        !moment_nm.isAllFinite()) { return false; }

    // Preserve collective as far as the physical rotor range permits.
    const float collective = limit(collective_n, N * kTmin, N * kTmax);
    float trim[N]{}, delta[N]{};
    float factor = 1.f;
    for (int i = 0; i < N; ++i) {
        trim[i] = config.physical_inverse[i][3] * collective;
        for (int axis = 0; axis < 3; ++axis) {
            delta[i] += config.physical_inverse[i][axis] * moment_nm(axis);
        }
        if (trim[i] < kTmin - 1e-4f || trim[i] > kTmax + 1e-4f) { return false; }
        if (delta[i] > 0.f) {
            factor = std::min(factor, (kTmax - trim[i]) / delta[i]);
        } else if (delta[i] < 0.f) {
            factor = std::min(factor, (kTmin - trim[i]) / delta[i]);
        }
    }
    factor = limit(factor, 0.f, 1.f);

    float u[N]{};
    for (int i = 0; i < N; ++i) {
        const float thrust = limit(trim[i] + factor * delta[i], kTmin, kTmax);
        const float discriminant = kC1 * kC1 + 4.f * kC2 * (thrust - kTmin);
        u[i] = limit((-kC1 + std::sqrt(discriminant)) / (2.f * kC2), 0.f, 1.f);
    }

    float command[N]{};
    for (int axis = 0; axis < N; ++axis) {
        for (int i = 0; i < N; ++i) {
            command[axis] += config.allocator[axis][i] * u[i];
        }
        command[axis] *= config.scale[axis];
        if (!std::isfinite(command[axis])) { return false; }
    }
    output.torque = Vector3{
        limit(command[0], -1.f, 1.f),
        limit(command[1], -1.f, 1.f),
        limit(command[2], -1.f, 1.f)
    };
    output.thrust_z = limit(command[3], -1.f, 0.f);
    output.moment_scale = factor;
    return true;
}

} // namespace se3_control
