# Generic plan loader; consumers retain flight-mode and motion authority.
px4_add_module(
    MODULE modules__trajectory_manager
    MAIN trajectory
    SRCS
        trajectory.cpp
    DEPENDS
        trajectory_generator
)
