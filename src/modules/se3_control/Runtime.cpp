// Geometric SE(3) control with body-rate and collective-thrust setpoints.
//
// The controller tracks a position and yaw reference using vehicle state.
// mc_rate_control tracks commanded body rates; control_allocator produces
// actuator commands. Commander manages flight-mode transitions and failsafes.
//
// Setpoints are published only while this registered flight mode is active.

#include "AttitudeLaws.hpp"
#include "Controller.hpp"
#include "PhysicalWrenchAdapter.hpp"

#include <trajectory/Sampler.hpp>
#include <trajectory/Plan.hpp>
#include <px4_platform_common/atomic.h>
#include <uORB/topics/vehicle_command.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/se3_trajectory_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <matrix/matrix/math.hpp>

// #include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

using namespace time_literals;

namespace
{
constexpr char kModeName[] = "SE3";
constexpr float kGravity = 9.80665f;
constexpr hrt_abstime kPositionTimeout = 200_ms;
constexpr hrt_abstime kAttitudeTimeout = 200_ms;
constexpr hrt_abstime kGyroTimeout = 100_ms;

bool fresh(hrt_abstime now, hrt_abstime stamp, hrt_abstime limit)
{
    return stamp > 0 && now >= stamp && now - stamp <= limit;
}
} // namespace

class Se3Runtime final : public ModuleBase<Se3Runtime>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    Se3Runtime() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) {}

    ~Se3Runtime() override
    {
        // Unregister the flight mode when the controller shuts down.
        // Commander handles any required mode transition.
        if (_mode_id >= 0 && _arming_check_id >= 0) {
            unregister_ext_component_s unregister{};
            unregister.timestamp = hrt_absolute_time();
            std::strncpy(unregister.name, kModeName, sizeof(unregister.name) - 1);
            unregister.arming_check_id = _arming_check_id;
            unregister.mode_id = _mode_id;
            unregister.mode_executor_id = -1;
            _unregister_pub.publish(unregister);
        }
    }

    bool init()
    {
        vehicle_status_s status{};

        if (_status_sub.copy(&status) && status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
            PX4_ERR("SE3 registration requires a disarmed vehicle");
            return false;
        }

        updateParams();
        _physical_config_valid = se3_control::PhysicalWrenchAdapter::load_configuration(_physical_config);
        _registration_request_id = hrt_absolute_time();

        register_ext_component_request_s request{};
        request.timestamp = _registration_request_id;
        request.request_id = _registration_request_id;
        request.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
        std::strncpy(request.name, kModeName, sizeof(request.name) - 1);
        request.register_arming_check = true;
        request.register_mode = true;
        request.register_mode_executor = false;
        request.enable_replace_internal_mode = false;
        request.activate_mode_immediately = false;
        _register_pub.publish(request);

        // ScheduleOnInterval(10_ms); // SE(3) reference update at 100 Hz.
        ScheduleOnInterval(4_ms);
        return true;
    }

    int print_status() override
    {
        PX4_INFO("Commander mode id=%d arming check id=%d", (int)_mode_id, (int)_arming_check_id);
        PX4_INFO("enabled=%d active=%d faulted=%d", (int)(_param_enable.get() == 1),
                 (int)_active.load(), (int)_faulted);
        PX4_INFO("trajectory: %s", _executing.load() ? "running" :
                 (_completed.load() ? "completed (holding final pose)" : "holding"));
        PX4_INFO("torque law=%ld (0=normalized, 1=physical)", (long)_param_torque_law.get());
        return 0;
    }

    static int task_spawn(int argc, char *argv[])
    {
        auto *instance = new Se3Runtime();

        if (instance) {
            _object.store(instance);
            _task_id = task_id_is_work_queue;

            if (instance->init()) {
                return PX4_OK;
            }
        }

        delete instance;
        _object.store(nullptr);
        _task_id = -1;
        return PX4_ERROR;
    }

    static int custom_command(int argc, char *argv[])
    {
        Se3Runtime *instance = get_instance();

        if (!instance) {
            PX4_ERR("start se3_control first");
            return PX4_ERROR;
        }

        if (argc == 1 && std::strcmp(argv[0], "select") == 0) {
            return instance->command_select();
        }

        if (argc == 1 && std::strcmp(argv[0], "hold") == 0) {
            return instance->command_hold();
        }

        return print_usage("unknown command");
    }

    static int print_usage(const char *reason = nullptr)
    {
        if (reason) { PX4_WARN("%s", reason); }
        PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Experimental SE(3) geometric body-rate controller for an already-flying multicopter.
Registered at SITL boot. Selection requires configured, valid airborne state. No autonomous takeoff.
PX4 retains rate PID, allocator, and output/flight-safety ownership.
)DESCR_STR");
        PRINT_MODULE_USAGE_NAME("se3_control", "controller");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_COMMAND("select");
        PRINT_MODULE_USAGE_COMMAND("hold");
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
        return 0;
    }

private:
    // CLI threads may request a transition, but only Run() modifies the
    // reference and trajectory clock on the controller work queue.
    int command_select()
    {
        // Commander owns mode transitions and can reject this request.
        const int mode_id = _registered_mode_id.load();
        if (mode_id < 0) {
            PX4_ERR("SE3 has not registered with Commander");
            return PX4_ERROR;
        }

        uORB::Subscription status_sub{ORB_ID(vehicle_status)};
        vehicle_status_s status{};
        if (!status_sub.copy(&status)) {
            PX4_ERR("vehicle status unavailable");
            return PX4_ERROR;
        }

        vehicle_command_s command{};
        command.timestamp = hrt_absolute_time();
        command.command = vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE;
        command.param1 = static_cast<float>(mode_id);
        command.source_system = status.system_id;
        command.source_component = status.component_id;
        command.target_system = status.system_id;
        command.target_component = status.component_id;

        if (!_vehicle_command_pub.publish(command)) {
            PX4_ERR("mode request publication failed");
            return PX4_ERROR;
        }

        PX4_INFO("requested SE3 mode %d", mode_id);
        return PX4_OK;
    }

    int command_hold()
    {
        if (!_active.load()) {
            PX4_ERR("SE3 is not the active flight mode");
            return PX4_ERROR;
        }

        _hold_requested.store(true);
        PX4_INFO("SE3 current-position hold requested");
        return PX4_OK;
    }

    void capture_hold()
    {
        // Initialize a stationary reference at the measured local pose.
        _origin = {};
        _origin.position = {_position.x, _position.y, _position.z};
        _origin.yaw = _position.heading;
        _sequence = trajectory_generator::Sequence{};
        _sequence.count = 1;
        _sequence.segments[0].type = trajectory_generator::Segment::Type::Hold;
        _sequence.segments[0].duration_s = 86400.0f;
        _start_time = hrt_absolute_time();
        _executing.store(false);
        _completed.store(false);
        _active_plan_crc = 0;
    }

    void Run() override
    {
        if (should_exit()) {
            trajectory_generator::set_se3_start_allowed(false);
            ScheduleClear();
            exit_and_cleanup();
            return;
        }

        if (_parameter_update_sub.updated()) {
            parameter_update_s update{};
            _parameter_update_sub.copy(&update);
            updateParams();
            _physical_config_valid = se3_control::PhysicalWrenchAdapter::load_configuration(_physical_config);
        }

        // Commander assigns both IDs. Matching the request ID prevents
        // accepting another component's registration reply.
        if (_mode_id < 0) {
            register_ext_component_reply_s response{};

            if (_register_reply_sub.update(&response) &&
                response.request_id == _registration_request_id &&
                std::strncmp(response.name, kModeName, sizeof(response.name)) == 0) {
                if (response.success && response.mode_id >= 0 && response.arming_check_id >= 0) {
                    _mode_id = response.mode_id;
                    _registered_mode_id.store(response.mode_id);
                    _arming_check_id = response.arming_check_id;

                    vehicle_control_mode_s config{};
                    config.timestamp = hrt_absolute_time();
                    config.source_id = static_cast<uint8_t>(_mode_id);
                    // Only the PX4 rate loop and allocator are enabled.
                    // Commander derives multicopter-position control from
                    // the position/velocity/altitude flags (all remain false).
                    config.flag_control_rates_enabled = false; // true for rate pid enabled
                    config.flag_control_allocation_enabled = true;
                    _mode_control_config_pub.publish(config);
                    PX4_INFO("registered mode %d (arming check %d)", (int)_mode_id, (int)_arming_check_id);
                } else {
                    PX4_ERR("Commander mode registration rejected");
                }
            }
        }

        _status_sub.update(&_status);
        _control_mode_sub.update(&_control_mode);
        if (_landed_sub.update(&_landed)) { _landed_received = true; }
        _position_sub.update(&_position);
        _attitude_sub.update(&_attitude);
        _gyro_sub.update(&_gyro);

        const bool valid = ready();

        // Commander broadcasts health requests ~3 Hz. Reply even while
        // disabled; lack of a reply would cause a different timeout path.
        arming_check_request_s request{};
        if (_arming_check_id >= 0 && _arming_request_sub.update(&request)) {
            arming_check_reply_s reply{};
            reply.timestamp = hrt_absolute_time();
            reply.request_id = request.request_id;
            reply.registration_id = static_cast<uint8_t>(_arming_check_id);
            reply.can_arm_and_run = valid;
            reply.mode_req_angular_velocity = true;
            reply.mode_req_attitude = true;
            reply.mode_req_local_position = true;
            reply.mode_req_prevent_arming = true; // Airborne handoff only, no takeoff/arm.
            _arming_reply_pub.publish(reply);
        }

        // No publication at all while another flight mode owns the setpoint.
        // const bool selected = _mode_id >= 0 && _status.nav_state == _mode_id &&
        //                       _control_mode.flag_control_rates_enabled &&
        //                       !_control_mode.flag_control_attitude_enabled &&
        //                       !_control_mode.flag_multicopter_position_control_enabled &&
        //                       _control_mode.flag_control_allocation_enabled;

        // use this for normalized torque
        const bool selected = _mode_id >= 0 && _status.nav_state == _mode_id &&
                              !_control_mode.flag_control_rates_enabled &&
                              !_control_mode.flag_control_attitude_enabled &&
                              !_control_mode.flag_multicopter_position_control_enabled &&
                              _control_mode.flag_control_allocation_enabled;

        if (!selected) {
            trajectory_generator::set_se3_start_allowed(false);
            _active.store(false);
            _executing.store(false);
            _completed.store(false);
            _hold_requested.store(false);

            // Keep faults latched until disarm, including after a mode change.
            if (_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED) {
                _faulted = false;
            }

            return;
        }

        // Do not resume control just because a stale sample recovered or
        // the estimator briefly became valid again. Force a mode exit.
        if (!valid) {
            trajectory_generator::set_se3_start_allowed(false);
            _active.store(false);
            _executing.store(false);
            _faulted = true;
            return;
        }

        if (!_active.load()) {
            // Mode entry is deliberately passive: capture the actual pose
            // and hold it until an explicit trajectory start arrives.
            capture_hold();
            _hold_requested.store(false);
            _xy_reset_counter = _position.xy_reset_counter;
            _z_reset_counter = _position.z_reset_counter;
            _quat_reset_counter = _attitude.quat_reset_counter;
            _active.store(true);
        }

        // Estimator-frame discontinuities are not silently interpreted as
        // trajectory tracking errors. Commander must reject this mode.
        if (_xy_reset_counter != _position.xy_reset_counter ||
            _z_reset_counter != _position.z_reset_counter ||
            _quat_reset_counter != _attitude.quat_reset_counter) {
            trajectory_generator::set_se3_start_allowed(false);
            _active.store(false);
            _executing.store(false);
            _faulted = true;
            return;
        }

        // Only the PX4 control work queue changes the active reference.
        // The trajectory CLI validates and queues a plan atomically through
        // the existing shared trajectory library; it cannot select a mode.
        bool pending = true;

        if (_hold_requested.compare_exchange(&pending, false)) {
            trajectory_generator::set_se3_start_allowed(false);
            capture_hold();
        }

        trajectory_generator::Plan &plan = _pending_plan;

        if (trajectory_generator::consume_start_request(plan)) {
            // Explicit pilot request. Deliberately do not compare the current
            // position or yaw with the plan origin, and do not reposition first.
            _origin = plan.origin;
            _sequence = plan.sequence;
            _duration_s = plan.duration_s;
            _active_plan_crc = plan.checksum;
            ++_run_id;
            _start_time = hrt_absolute_time();
            _executing.store(true);
            _completed.store(false);
            PX4_INFO("SE3 trajectory started (%u segments)",
                     static_cast<unsigned>(_sequence.count));
        }

        // Completion makes the existing sampler hold the final reference.
        // A new start can be requested only once SE3 is holding again.
        trajectory_generator::set_se3_start_allowed(
            !_executing.load() && !_hold_requested.load());

        const float elapsed_s =
            static_cast<float>(hrt_absolute_time() - _start_time) * 1.0e-6f;
        trajectory_generator::Reference reference{};

        if (!trajectory_generator::sample(_sequence, elapsed_s, _origin, reference)) {
            _active.store(false);
            _executing.store(false);
            trajectory_generator::set_se3_start_allowed(false);
            _faulted = true;
            return;
        }

        if (_executing.load() && elapsed_s >= _duration_s) {
            _executing.store(false);
            _completed.store(true); // Sampler holds the exact final reference.
        }

        se3_control::State state{};
        state.position = se3_control::Vector3{_position.x, _position.y, _position.z};
        state.velocity = se3_control::Vector3{_position.vx, _position.vy, _position.vz};
        state.angular_velocity = se3_control::Vector3{_gyro.xyz[0], _gyro.xyz[1], _gyro.xyz[2]};

        const matrix::Quatf quat{_attitude.q};
        state.attitude = matrix::Dcmf{quat};

        const float mass = _param_mass.get();
        se3_control::TranslationGains translation_gains{};
        translation_gains.mass = mass;
        translation_gains.position = mass * _param_kx.get();
        translation_gains.velocity = mass * _param_kv.get();

        /////////////////////// SO(3) attitude + rate PID //
        // se3_control::GeometricOutput geometric{};
        // se3_control::Vector3 body_rate{};
        // const se3_control::Vector3 attitude_gains{
        //     _param_krx.get(), _param_kry.get(), _param_krz.get()
        // };

        // if (!se3_control::Controller::compute_geometry(state, reference, translation_gains, geometric) ||
        //     !se3_control::AttitudeLaws::body_rate(state, geometric.desired_attitude, attitude_gains, body_rate)) {
        //     _active.store(false);
        //     _executing.store(false);
        //     trajectory_generator::set_se3_start_allowed(false);
        //     _faulted = true;
        //     return;
        // }

        // // Convert collective thrust to the normalized PX4 body-z convention.
        // const float thrust = _param_hover.get() * geometric.collective_thrust_n / (mass * kGravity);
        // // const float specific_thrust = geometric.collective_thrust_n / mass;
        // // constexpr float kThrustSlope  = 0.03749f;
        // // constexpr float kThrustOffset = 0.22043f;
        // // const float thrust = kThrustSlope * specific_thrust + kThrustOffset;
        // const float rate_limit = _param_rate_max.get();

        // if (!std::isfinite(thrust) || !body_rate.isAllFinite() || !std::isfinite(rate_limit)) {
        //     _active.store(false);
        //     _executing.store(false);
        //     trajectory_generator::set_se3_start_allowed(false);
        //     _faulted = true;
        //     return;
        // }

        // vehicle_rates_setpoint_s setpoint{};
        // setpoint.timestamp = hrt_absolute_time();
        // setpoint.roll = fminf(fmaxf(body_rate(0), -rate_limit), rate_limit);
        // setpoint.pitch = fminf(fmaxf(body_rate(1), -rate_limit), rate_limit);
        // setpoint.yaw = fminf(fmaxf(body_rate(2), -rate_limit), rate_limit);
        // setpoint.thrust_body[0] = 0.f;
        // setpoint.thrust_body[1] = 0.f;
        // setpoint.thrust_body[2] = -fminf(fmaxf(thrust, 0.0f), 1.0f);
        // _rate_setpoint_pub.publish(setpoint);
        /////////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////// torque mode block
        se3_control::GeometricOutput geometric{};
        bool control_ok = se3_control::Controller::compute_geometry(state, reference, translation_gains, geometric);
        se3_control::Vector3 normalized_torque{};

        // Existing collective-thrust mapping.
        // Used when SE3_TORQUE_LAW == 0.
        float thrust = _param_hover.get() * geometric.collective_thrust_n / (mass * kGravity);

        if (control_ok && _param_torque_law.get() == 0) {
            // --------------------------------------
            // Controller 0: Direct normalized SO(3)
            // --------------------------------------
            se3_control::NormalizedTorqueOutput output{};
            se3_control::NormalizedAttitudeGains gains{};

            gains.attitude = se3_control::Vector3{0.15f * _param_krx.get(), 0.15f * _param_kry.get(), 0.20f * _param_krz.get()};
            gains.angular_velocity = se3_control::Vector3{0.15f, 0.15f, 0.20f};
            gains.angular_acceleration = se3_control::Vector3{0.f, 0.f, 0.f};

            control_ok = se3_control::AttitudeLaws::normalized(state, geometric.desired_attitude, gains, output);
            if (control_ok) {
                normalized_torque = output.torque;
            }

        } else if (control_ok && _param_torque_law.get() == 1) {
            // --------------------------------------
            // Controller 1: Lee physical SO(3)
            // --------------------------------------

            // F450 rigid-body inertia [kg m^2]
            se3_control::InertiaMatrix inertia{};
            inertia(0, 0) = _param_jxx.get();
            inertia(1, 1) = _param_jyy.get();
            inertia(2, 2) = _param_jzz.get();

            se3_control::PhysicalAttitudeGains gains{};
            gains.attitude = se3_control::Vector3{_param_ph_krx.get(), _param_ph_kry.get(), _param_ph_krz.get()};
            gains.angular_velocity = se3_control::Vector3{_param_ph_kwx.get(), _param_ph_kwy.get(), _param_ph_kwz.get()};

            // Physical moment [N m]
            se3_control::Vector3 moment_nm{};

            control_ok = se3_control::AttitudeLaws::physical(state, geometric.desired_attitude, inertia, gains, moment_nm);

            // Convert the physical wrench to PX4's
            // normalized torque and thrust coordinates.
            se3_control::PhysicalWrenchAdapter::Output adapted{};

            if (control_ok && _physical_config_valid) {
                control_ok = se3_control::PhysicalWrenchAdapter::adapt(_physical_config, geometric.collective_thrust_n, moment_nm, adapted);
            } else {
                control_ok = false;
            }
            if (control_ok) {
                normalized_torque = adapted.torque;
                thrust = -adapted.thrust_z;
            }
        } else {
            control_ok = false;
        }

        // --------------------------------------
        // Common validation and PX4 publication
        // --------------------------------------
        if (!control_ok ||
            !std::isfinite(thrust) ||
            !normalized_torque.isAllFinite()) {

            _active.store(false);
            _executing.store(false);
            trajectory_generator::set_se3_start_allowed(false);
            _faulted = true;
            return;
        }

        const uint64_t now = hrt_absolute_time();

        // Publish thrust before torque.
        // PX4's control allocator is triggered by torque updates.
        vehicle_thrust_setpoint_s thrust_sp{};
        thrust_sp.timestamp = now;
        thrust_sp.timestamp_sample = _gyro.timestamp_sample;
        thrust_sp.xyz[2] = -fminf(fmaxf(thrust, 0.f), 1.f);
        _thrust_setpoint_pub.publish(thrust_sp);

        vehicle_torque_setpoint_s torque_sp{};
        torque_sp.timestamp = now;
        torque_sp.timestamp_sample = _gyro.timestamp_sample;
        for (int axis = 0; axis < 3; ++axis) {
            torque_sp.xyz[axis] = fminf(fmaxf(normalized_torque(axis), -1.f), 1.f);
        }
        _torque_setpoint_pub.publish(torque_sp);
        ///////////////////////////////////////////////////////////////////////////////

        // Expose the exact geometric attitude reference through the standard
        // PX4 observation topic. SE3 still owns body-rate control: Commander
        // keeps mc_att_control disabled in this external flight mode.
        // Rotation maps body FRD to world NED, matching PX4 q_d convention.
        const matrix::Quatf desired_quaternion{geometric.desired_attitude.attitude};
        vehicle_attitude_setpoint_s attitude_sp{};
        // attitude_sp.timestamp = setpoint.timestamp; // for SO3 attitude + rate PID
        attitude_sp.timestamp = now;
        for (int component = 0; component < 4; ++component) {
            attitude_sp.q_d[component] = desired_quaternion(component);
        }
        attitude_sp.yaw_sp_move_rate = reference.yaw_rate;
        for (int axis = 0; axis < 3; ++axis) {
            // attitude_sp.thrust_body[axis] = setpoint.thrust_body[axis]; // for SO3 attitude + rate PID
            attitude_sp.thrust_body[axis] = thrust_sp.xyz[axis];
        }
        _attitude_setpoint_pub.publish(attitude_sp);

        // Record the exact reference sent to the geometric controller.
        se3_trajectory_status_s record{};
        // record.timestamp = setpoint.timestamp; // for SO3 attitude + rate PID
        record.timestamp = now;
        record.run_id = _run_id;
        record.plan_crc32 = _active_plan_crc;
        record.phase = _completed.load() ? se3_trajectory_status_s::PHASE_COMPLETED :
                       (_executing.load() ? se3_trajectory_status_s::PHASE_RUNNING :
                                            se3_trajectory_status_s::PHASE_HOLD);
        record.elapsed_s = _active_plan_crc > 0 ? fminf(elapsed_s, _duration_s) : 0.0f;
        for (int axis = 0; axis < 3; ++axis) {
            record.position[axis] = reference.position[axis];
            record.velocity[axis] = reference.velocity[axis];
            record.acceleration[axis] = reference.acceleration[axis];
            record.jerk[axis] = reference.jerk[axis];
            record.snap[axis] = reference.snap[axis];
        }
        record.yaw = reference.yaw;
        record.yaw_rate = reference.yaw_rate;
        record.yaw_acceleration = reference.yaw_acceleration;
        // Publish the same reference used for SE3 control on PX4's standard
        // trajectory topic. The FlightModeManager has no flight task in an
        // external mode; mc_pos_control remains disabled by Commander.
        // No controller state, arm state, or trajectory lifecycle is changed.
        trajectory_setpoint_s trajectory_sp{};
        trajectory_sp.timestamp = record.timestamp;
        for (int axis = 0; axis < 3; ++axis) {
            trajectory_sp.position[axis] = reference.position[axis];
            trajectory_sp.velocity[axis] = reference.velocity[axis];
            trajectory_sp.acceleration[axis] = reference.acceleration[axis];
            trajectory_sp.jerk[axis] = reference.jerk[axis];
        }
        trajectory_sp.yaw = reference.yaw;
        trajectory_sp.yawspeed = reference.yaw_rate;
        _trajectory_pub.publish(trajectory_sp);

        _trajectory_status_pub.publish(record);
    }

    bool ready() const
    {
        const hrt_abstime now = hrt_absolute_time();
        const bool flying = _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
                            !_landed.landed && !_status.failsafe &&
                            _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
                            !_status.is_vtol;
        const bool valid_position = _position.xy_valid && _position.z_valid &&
                                    _position.v_xy_valid && _position.v_z_valid &&
                                    std::isfinite(_position.x) && std::isfinite(_position.y) &&
                                    std::isfinite(_position.z) && std::isfinite(_position.vx) &&
                                    std::isfinite(_position.vy) && std::isfinite(_position.vz) &&
                                    std::isfinite(_position.heading);
        const bool valid_attitude = std::isfinite(_attitude.q[0]) && std::isfinite(_attitude.q[1]) &&
                                    std::isfinite(_attitude.q[2]) && std::isfinite(_attitude.q[3]);
        const float norm2 = _attitude.q[0] * _attitude.q[0] +
                             _attitude.q[1] * _attitude.q[1] +
                             _attitude.q[2] * _attitude.q[2] +
                             _attitude.q[3] * _attitude.q[3];
        const bool valid_gyro = std::isfinite(_gyro.xyz[0]) && std::isfinite(_gyro.xyz[1]) &&
                                std::isfinite(_gyro.xyz[2]);
        const bool supported_airframe = _param_autostart.get() != 6010 && _param_autostart.get() != 6002;
        const bool configured = _param_enable.get() == 1 && (
                                _param_torque_law.get() == 0 || (
                                    _param_torque_law.get() == 1 &&
                                    _physical_config_valid &&
                                    _param_jxx.get() > 0.f &&
                                    _param_jyy.get() > 0.f &&
                                    _param_jzz.get() > 0.f &&
                                    _param_ph_krx.get() > 0.f &&
                                    _param_ph_kry.get() > 0.f &&
                                    _param_ph_krz.get() > 0.f &&
                                    _param_ph_kwx.get() > 0.f &&
                                    _param_ph_kwy.get() > 0.f &&
                                    _param_ph_kwz.get() > 0.f
                                )) &&
                                _param_mass.get() > 0.0f &&
                                _param_kx.get() > 0.0f && _param_kv.get() > 0.0f &&
                                _param_krx.get() > 0.0f && _param_kry.get() > 0.0f &&
                                _param_krz.get() > 0.0f &&
                                _param_rate_max.get() > 0.0f &&
                                _param_hover.get() > 0.0f && _param_hover.get() < 1.0f &&
                                std::isfinite(_param_mass.get()) &&
                                std::isfinite(_param_kx.get()) && std::isfinite(_param_kv.get()) &&
                                std::isfinite(_param_krx.get()) && std::isfinite(_param_kry.get()) &&
                                std::isfinite(_param_krz.get()) &&
                                std::isfinite(_param_rate_max.get()) && std::isfinite(_param_hover.get());

        return _mode_id >= 0 && !_faulted && _landed_received && flying &&
               supported_airframe &&
               valid_position && valid_attitude &&
               norm2 > 0.9f && norm2 < 1.1f && valid_gyro && configured &&
               fresh(now, _position.timestamp, kPositionTimeout) &&
               fresh(now, _attitude.timestamp, kAttitudeTimeout) &&
               fresh(now, _gyro.timestamp, kGyroTimeout);
    }

    se3_control::PhysicalWrenchAdapter::Configuration _physical_config{};
    bool _physical_config_valid{false};

    // Commander registration, flight-mode state, and rate setpoint interfaces.
    uORB::Publication<register_ext_component_request_s> _register_pub{ORB_ID(register_ext_component_request)};
    uORB::Subscription _register_reply_sub{ORB_ID(register_ext_component_reply)};
    uORB::Publication<vehicle_control_mode_s> _mode_control_config_pub{ORB_ID(config_control_setpoints)};
    uORB::Publication<unregister_ext_component_s> _unregister_pub{ORB_ID(unregister_ext_component)};
    uORB::Subscription _arming_request_sub{ORB_ID(arming_check_request)};
    uORB::Publication<arming_check_reply_s> _arming_reply_pub{ORB_ID(arming_check_reply)};

    uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::Subscription _landed_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _gyro_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Publication<vehicle_rates_setpoint_s> _rate_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
    uORB::Publication<vehicle_torque_setpoint_s> _torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)}; // thrust and torque
    uORB::Publication<vehicle_thrust_setpoint_s> _thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
    uORB::Publication<se3_trajectory_status_s> _trajectory_status_pub{ORB_ID(se3_trajectory_status)};
    uORB::Publication<trajectory_setpoint_s> _trajectory_pub{ORB_ID(trajectory_setpoint)};

    vehicle_status_s _status{};
    vehicle_control_mode_s _control_mode{};
    vehicle_land_detected_s _landed{};
    bool _landed_received{false};
    bool _faulted{false};
    vehicle_local_position_s _position{};
    vehicle_attitude_s _attitude{};
    vehicle_angular_velocity_s _gyro{};

    trajectory_generator::Reference _origin{};
    trajectory_generator::Sequence _sequence{};
    trajectory_generator::Plan _pending_plan{};
    hrt_abstime _start_time{0};
    uint64_t _registration_request_id{0};
    int8_t _mode_id{-1};
    int8_t _arming_check_id{-1};
    uint8_t _xy_reset_counter{0};
    uint8_t _z_reset_counter{0};
    uint8_t _quat_reset_counter{0};
    float _duration_s{0.0f};
    uint32_t _run_id{0};
    uint32_t _active_plan_crc{0};
    px4::atomic_int32_t _registered_mode_id{-1};
    uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
    px4::atomic_bool _active{false};
    px4::atomic_bool _executing{false};
    px4::atomic_bool _completed{false};
    px4::atomic_bool _hold_requested{false};

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SE3_ENABLE>) _param_enable,
        (ParamFloat<px4::params::SE3_MASS>) _param_mass,
        (ParamFloat<px4::params::SE3_KX>) _param_kx,
        (ParamFloat<px4::params::SE3_KV>) _param_kv,
        (ParamFloat<px4::params::SE3_KRX>) _param_krx,
        (ParamFloat<px4::params::SE3_KRY>) _param_kry,
        (ParamFloat<px4::params::SE3_KRZ>) _param_krz,
        (ParamFloat<px4::params::SE3_RATE_MAX>) _param_rate_max,
        (ParamFloat<px4::params::MPC_THR_HOVER>) _param_hover,
        (ParamInt<px4::params::SYS_AUTOSTART>) _param_autostart,

        (ParamInt<px4::params::SE3_TORQUE_LAW>) _param_torque_law,
        (ParamFloat<px4::params::SE3_JXX>) _param_jxx,
        (ParamFloat<px4::params::SE3_JYY>) _param_jyy,
        (ParamFloat<px4::params::SE3_JZZ>) _param_jzz,
        (ParamFloat<px4::params::SE3_PH_KRX>) _param_ph_krx,
        (ParamFloat<px4::params::SE3_PH_KRY>) _param_ph_kry,
        (ParamFloat<px4::params::SE3_PH_KRZ>) _param_ph_krz,
        (ParamFloat<px4::params::SE3_PH_KWX>) _param_ph_kwx,
        (ParamFloat<px4::params::SE3_PH_KWY>) _param_ph_kwy,
        (ParamFloat<px4::params::SE3_PH_KWZ>) _param_ph_kwz
    )
};

extern "C" __EXPORT int se3_control_main(int argc, char *argv[])
{
    // Allow shutdown only when disarmed; Commander handles a lost mode.
    if (argc > 1 && std::strcmp(argv[1], "stop") == 0) {
        uORB::Subscription status_sub{ORB_ID(vehicle_status)};
        vehicle_status_s status{};

        if (!status_sub.copy(&status) ||
            status.arming_state != vehicle_status_s::ARMING_STATE_DISARMED) {
            PX4_ERR("SE3 can only be stopped while disarmed");
            return PX4_ERROR;
        }
    }

    return Se3Runtime::main(argc, argv);
}
