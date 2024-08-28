cmake_minimum_required(VERSION 3.14)
project(sensing_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(sensing_interface
  src/carla_sensing_interface_node.cpp
  src/carla_sensing_interface.cpp
)

ament_auto_package(
    INSTALL_TO_SHARE 
    launch
)
