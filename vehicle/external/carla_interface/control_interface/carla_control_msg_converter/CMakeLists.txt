cmake_minimum_required(VERSION 3.14)
project(carla_control_msg_converter)

find_package(autoware_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
# find_package(autoware_auto_control_msgs REQUIRED)

autoware_package()

set(CARLA_CONTROL_MSG_CONVERTER carla_control_msg_converter)
ament_auto_add_library(${CARLA_CONTROL_MSG_CONVERTER} SHARED
  src/carla_control_msg_converter.cpp
)

# target_compile_options(${CARLA_CONTROL_MSG_CONVERTER} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${CARLA_CONTROL_MSG_CONVERTER}
  PLUGIN "CarlaControlMsgConverter"
  EXECUTABLE ${CARLA_CONTROL_MSG_CONVERTER}_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
