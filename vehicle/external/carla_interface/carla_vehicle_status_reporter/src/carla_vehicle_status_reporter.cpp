// CarlaEgoVehicleStatus to AckermannControlDrive
// cf) carla_control_interface의 carla_ackermann_controller는 ros-bridge의 ackermanncontrol 패키지를 쓰지 않고,
//     바로 AckermannControlCommand형식으로부터 CarlaEgoVehicleControl 메세지 만드는 패키지임. 
//     ==> 이것도 나중에 수정해서 사용해보고 싶음. (두 노드 쓰는 것보다 이게 편리하니까? && cpp니까?)

#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include <string>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_info.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

using carla_msgs::msg::CarlaEgoVehicleStatus;
using carla_msgs::msg::CarlaEgoVehicleInfo;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using autoware_auto_vehicle_msgs::msg::ControlModeReport;

class CarlaVehicleStatusReporter : public rclcpp::Node
{
public:
  rclcpp::Subscription<CarlaEgoVehicleStatus>::SharedPtr sub_vehicle_status_;
  rclcpp::Subscription<CarlaEgoVehicleInfo>::SharedPtr sub_vehicle_info_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vehicle_odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_control_auto_mode_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_status_;
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_status_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr pub_control_mode_;

  explicit CarlaVehicleStatusReporter(const rclcpp::NodeOptions & node_options) 
  : Node("CarlaVehicleStatusReporter", node_options),
  base_frame_id_(this->declare_parameter<std::string>("base_frame_id", "base_link"))
  {
    using std::placeholders::_1;

    const std::string role_name = declare_parameter<std::string>("role_name", "ego_vehicle");
    // base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    vehicle_info_updated_ = false;

    max_steering_angle_deg_ = declare_parameter<float>("max_steering_angle_deg", 70.0);

    sub_vehicle_status_ = create_subscription<CarlaEgoVehicleStatus>(
      "/carla/" + role_name + "/vehicle_status", rclcpp::QoS{1}, std::bind(&CarlaVehicleStatusReporter::onVehicleStatus, this, _1));
    sub_vehicle_info_ = create_subscription<CarlaEgoVehicleInfo>(
      "/carla/" + role_name + "/vehicle_info", rclcpp::QoS{1}, std::bind(&CarlaVehicleStatusReporter::onVehicleInfo, this, _1));
    sub_vehicle_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/carla/" + role_name + "/odometry", rclcpp::QoS{1}, std::bind(&CarlaVehicleStatusReporter::onVehicleOdom, this, _1));
    sub_control_auto_mode_ = create_subscription<std_msgs::msg::Bool>(
      "/carla/" + role_name + "/vehicle_control_manual_override", rclcpp::QoS{1}, std::bind(&CarlaVehicleStatusReporter::onControlAutoMode, this, _1));
    
    pub_steering_status_ = create_publisher<SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
    pub_velocity_status_ = create_publisher<VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
    pub_control_mode_ = create_publisher<ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  }
  virtual ~CarlaVehicleStatusReporter() {}

private:
  CarlaEgoVehicleInfo::SharedPtr vehicle_info_;
  std::string base_frame_id_;
  float max_steering_angle_deg_;
  bool vehicle_info_updated_;

  float get_vehicle_max_steering_angle(){
    // Get the maximum steering angle of a carla vehicle
    // :return: maximum steering angle [radians]

    // 70 degrees is the default max steering angle of a car
    float max_steering_angle = max_steering_angle_deg_ * (M_PI/180);

    //get max steering angle (use smallest non-zero value of all wheels)
    if (vehicle_info_updated_ == true){
      for (const auto & wheel : vehicle_info_->wheels) {
        if(wheel.max_steer_angle){
          if(wheel.max_steer_angle && wheel.max_steer_angle < max_steering_angle){
            max_steering_angle = wheel.max_steer_angle;
          }
        }
      }
    }
    return max_steering_angle;
  }

  void onVehicleInfo(const CarlaEgoVehicleInfo::SharedPtr msg)
  {
    vehicle_info_ = msg;
    vehicle_info_updated_ = true;
  }  
  void onVehicleStatus(const CarlaEgoVehicleStatus::SharedPtr msg)
  {
    SteeringReport steer_msg;

    // steer_msg.stamp = msg->header.stamp;
    steer_msg.stamp = this->get_clock()->now();
    steer_msg.steering_tire_angle = -(msg -> control.steer) * get_vehicle_max_steering_angle(); // autoware와 carla steer 부호 반대임. [radian]
    pub_steering_status_->publish(steer_msg);
  }
  void onVehicleOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    VelocityReport vel_msg;
    vel_msg.header = msg->header;
    vel_msg.header.stamp = this->get_clock()->now();
    vel_msg.header.frame_id = "base_link";
    vel_msg.longitudinal_velocity = msg->twist.twist.linear.x;
    vel_msg.lateral_velocity = msg->twist.twist.linear.y;
    vel_msg.heading_rate = msg->twist.twist.angular.z;
    pub_velocity_status_->publish(vel_msg);
  }
  void onControlAutoMode(const std_msgs::msg::Bool::SharedPtr msg)
  {
    ControlModeReport control_mode_msg_;
    control_mode_msg_.stamp = this->get_clock()->now();
    control_mode_msg_.mode = ControlModeReport::AUTONOMOUS;

    if(msg->data) control_mode_msg_.mode = ControlModeReport::MANUAL;;

    pub_control_mode_->publish(control_mode_msg_);
  } 
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CarlaVehicleStatusReporter)
