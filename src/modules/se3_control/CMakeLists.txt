# Geometric SE(3) body-rate control module.
px4_add_module(
    MODULE modules__se3_control
    MAIN se3_control
    SRCS
        Runtime.cpp
        Controller.cpp
        AttitudeReference.cpp
        AttitudeLaws.cpp
        PhysicalWrenchAdapter.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        trajectory_generator
        px4_work_queue
)
