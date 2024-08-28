cmake_minimum_required(VERSION 3.14)
project(carla_vehicle_status_reporter)

find_package(autoware_cmake REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)

autoware_package()

set(CARLA_VEHICLE_STATUS_REPORTER carla_vehicle_status_reporter)
ament_auto_add_library(${CARLA_VEHICLE_STATUS_REPORTER} SHARED
  src/carla_vehicle_status_reporter.cpp
)

# target_compile_options(${CARLA_VEHICLE_STATUS_REPORTER} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${CARLA_VEHICLE_STATUS_REPORTER}
  PLUGIN "CarlaVehicleStatusReporter"
  EXECUTABLE ${CARLA_VEHICLE_STATUS_REPORTER}_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
