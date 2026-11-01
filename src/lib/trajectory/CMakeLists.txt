# Shared analytic trajectory generator for native PX4 controllers.
# The library has no ROS 2 or YAML dependencies.

px4_add_library(trajectory_generator
  Sampler.cpp
  Plan.cpp
)

target_include_directories(trajectory_generator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
