# SE(3) reference and execution status at the controller update time.
# Logged onboard; it is not part of the uXRCE-DDS topic configuration.
uint64 timestamp                  # PX4 boot time [us]
uint32 run_id                     # Increments on each accepted run command
uint32 plan_crc32                 # Checksum from the selected .traj file
uint8 PHASE_HOLD = 1
uint8 PHASE_RUNNING = 2
uint8 PHASE_COMPLETED = 3
uint8 phase                      # Reference execution phase
float32 elapsed_s                # Time since trajectory start, clamped at duration [s]
float32[3] position              # Desired position in PX4 local NED [m]
float32[3] velocity              # Desired velocity in PX4 local NED [m/s]
float32[3] acceleration          # Desired acceleration [m/s^2]
float32[3] jerk                  # Desired jerk [m/s^3]
float32[3] snap                  # Desired snap [m/s^4]
float32 yaw                      # Desired heading [rad]
float32 yaw_rate                 # Desired heading rate [rad/s]
float32 yaw_acceleration         # Desired heading acceleration [rad/s^2]
