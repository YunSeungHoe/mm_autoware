cmake_minimum_required(VERSION 3.14)
project(mode_change_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(mode_change_interface
  src/carla_mode_change_interface_node.cpp
  src/carla_mode_change_interface.cpp
)

ament_auto_package(
    INSTALL_TO_SHARE 
    launch
)
