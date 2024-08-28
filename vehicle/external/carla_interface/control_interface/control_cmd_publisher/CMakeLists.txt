cmake_minimum_required(VERSION 3.14)
project(control_cmd_publisher)

find_package(autoware_cmake REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)

autoware_package()

set(CONTROL_CMD_PUBLISHER control_cmd_publisher)
ament_auto_add_library(${CONTROL_CMD_PUBLISHER} SHARED
  src/control_cmd_publisher.cpp
)

# target_compile_options(${CONTROL_CMD_PUBLISHER} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${CONTROL_CMD_PUBLISHER}
  PLUGIN "ControlCmdPublisher"
  EXECUTABLE ${CONTROL_CMD_PUBLISHER}_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
