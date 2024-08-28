// Copyright 2021 TierIV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vehicle_velocity_converter/vehicle_velocity_converter.hpp"

VehicleVelocityConverter::VehicleVelocityConverter() : Node("vehicle_velocity_converter")
{
  // frame_id_ = declare_parameter("frame_id", "base_link");
  frame_id_ = declare_parameter("frame_id", "map");
  
  odm_sub = create_subscription<nav_msgs::msg::Odometry>(
    "/carla/ego_vehicle/odometry", rclcpp::QoS{100},
    std::bind(&VehicleVelocityConverter::callbackVelocityReport, this, std::placeholders::_1));

  velocity_report_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    // "velocity_status", 10);
    "velocity_status", rclcpp::QoS{100});
}

void VehicleVelocityConverter::callbackVelocityReport(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // if (msg->header.frame_id != frame_id_) {
  //   RCLCPP_WARN(get_logger(), "TEST MSG : frame_id is not ego_vehicle.");
  // }

  // set twist with covariance msg from vehicle report msg
  autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_msg;
  velocity_report_msg.header = msg->header;
  velocity_report_msg.header.stamp = this->get_clock()->now();
  velocity_report_msg.header.frame_id = "base_link";
  velocity_report_msg.longitudinal_velocity = msg->twist.twist.linear.x;
  velocity_report_msg.lateral_velocity = msg->twist.twist.linear.y;
  velocity_report_msg.heading_rate = msg->twist.twist.angular.z;
  velocity_report_pub_->publish(velocity_report_msg);
}
