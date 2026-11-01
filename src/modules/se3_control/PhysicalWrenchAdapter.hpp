#pragma once

#include "Types.hpp"

namespace se3_control
{

class PhysicalWrenchAdapter
{
public:
    struct Configuration {
        float physical_inverse[4][4]{};
        float allocator[4][4]{};
        float scale[4]{};
    };

    struct Output {
        Vector3 torque{};             // PX4 normalized roll/pitch/yaw
        float thrust_z{};             // PX4 normalized body-Z, negative
        float moment_scale{1.f};      // [0,1], physical actuator limit factor
    };

    // Read live CA_ROTOR* parameters once at startup / parameter update.
    static bool load_configuration(Configuration & config);

    // F450-only: physical [N, N*m] -> PX4 normalized thrust and torque.
    static bool adapt(const Configuration & config,
                      float collective_n,
                      const Vector3 & moment_nm,
                      Output & output);
};

} // namespace se3_control
