cmake_minimum_required(VERSION 3.14)
project(carla_ackermann_controller)

find_package(autoware_cmake REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(carla_ackermann_msgs REQUIRED)

autoware_package()

set(CARLA_ACKERMANN_CONTROLLER carla_ackermann_controller)
ament_auto_add_library(${CARLA_ACKERMANN_CONTROLLER} SHARED
  include/carla_ackermann_controller.hpp
  include/pid_controller.hpp
  src/carla_ackermann_controller.cpp
  src/carla_control_physics.cpp
)

rclcpp_components_register_node(${CARLA_ACKERMANN_CONTROLLER}
  # PLUGIN "autoware::motion::coyntrol::trajectory_follower_node::Controller"
  PLUGIN "CarlaAckermannController"
  EXECUTABLE ${CARLA_ACKERMANN_CONTROLLER}_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
