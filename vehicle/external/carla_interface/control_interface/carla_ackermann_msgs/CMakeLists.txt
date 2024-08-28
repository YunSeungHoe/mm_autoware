cmake_minimum_required(VERSION 3.14)

project(carla_ackermann_msgs)

find_package(ros_environment REQUIRED)

set(MSG_FILES
    EgoVehicleControlInfo.msg EgoVehicleControlCurrent.msg
    EgoVehicleControlMaxima.msg EgoVehicleControlStatus.msg
    EgoVehicleControlTarget.msg)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Apend "msg/" to each file name
set(TEMP_LIST "")
foreach(MSG_FILE ${MSG_FILES})
  list(APPEND TEMP_LIST "msg/${MSG_FILE}")
endforeach()
set(MSG_FILES ${TEMP_LIST})

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  carla_msgs
  ADD_LINTER_TESTS)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
