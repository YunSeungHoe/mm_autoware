#ifndef CARLA_ACKERMANN_CONTROLLER_HPP_
#define CARLA_ACKERMANN_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_info.hpp>
#include <carla_ackermann_msgs/msg/ego_vehicle_control_info.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <string>
#include <algorithm>
#include "pid_controller.hpp"

using carla_msgs::msg::CarlaEgoVehicleStatus; 
using carla_msgs::msg::CarlaEgoVehicleControl;  //output
using carla_msgs::msg::CarlaEgoVehicleInfo; // input
using geometry_msgs::msg::AccelWithCovarianceStamped; // input
using nav_msgs::msg::Odometry;  // input
using carla_ackermann_msgs::msg::EgoVehicleControlInfo; //output
using autoware_auto_control_msgs::msg::AckermannControlCommand; //input

class CarlaAckermannController : public rclcpp::Node
{
public:
    explicit CarlaAckermannController(const rclcpp::NodeOptions & node_options);
    virtual ~CarlaAckermannController() {}

private:
    // PID controller
    PIDController speed_controller_;
    PIDController accel_controller_;

    // global variables
    rclcpp::TimerBase::SharedPtr timer_control_;
    carla_ackermann_msgs::msg::EgoVehicleControlInfo ego_info_;
    carla_msgs::msg::CarlaEgoVehicleStatus vehicle_status_;
    carla_msgs::msg::CarlaEgoVehicleInfo vehicle_info_;
    bool use_speed_pid_;
    float last_ackermann_msg_received_sec_;
    float current_accel_;
    float current_velocity_;

    // ROS
    rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_; // save in ego_info_.target
    rclcpp::Subscription<CarlaEgoVehicleStatus>::SharedPtr sub_vehicle_status_;
    rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr sub_accel_;
    rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<CarlaEgoVehicleInfo>::SharedPtr sub_vehicle_info_; // for setting ego_info_.restrictons

    rclcpp::Publisher<CarlaEgoVehicleControl>::SharedPtr pub_control_cmd_;
    rclcpp::Publisher<EgoVehicleControlInfo>::SharedPtr pub_control_info_; //ego_info_
    
    // timer & subscription callbacks
    void callbackTimer();
    void onCmd(const AckermannControlCommand::SharedPtr);
    void onStatus(const CarlaEgoVehicleStatus::SharedPtr);
    void onAccel(const AccelWithCovarianceStamped::SharedPtr);
    void onOdom(const Odometry::SharedPtr);
    void onInfo(const CarlaEgoVehicleInfo::SharedPtr);
    void onInfoAlter();

    // main functions
    void updateCurrentValues();
    void vehicleControlCycle();
    void sendEgoVehicleControlInfoMsg();

    // additional functions
    // // (inner functions of vehicleControlCycle)
    void controlSteering();
    void controlStopAndReverse();
    void runSpeedControlLoop();
    void runAccelControlLoop();
    void updateDriveVehicleControlCommand();
    // // (inner functions of onCmd)
    void setTargetSteeringAngle(float t_steer);
    void setTargetSpeed(float t_speed);
    void setTargetAccel(float t_accel);
    void setTargetJerk(float t_jerk);


    // physics functions
    CarlaEgoVehicleInfo initVehicleInfo();
    float getVehicleLayOffEngineAcceleration(CarlaEgoVehicleInfo vehicle_info);
    float getEngineBrakeForce();
    float getVehicleMass(CarlaEgoVehicleInfo vehicle_info);
    float getVehicleDrivingImpedanceAcceleration(CarlaEgoVehicleInfo vehicle_info, CarlaEgoVehicleStatus vehicle_status, bool reverse);
    float getRollingResistanceForce(CarlaEgoVehicleInfo vehicle_info);
    float getWeightForce(CarlaEgoVehicleInfo vehicle_info);
    float getAccelerationOfGravity();
    float getAerodynamicDragForce(CarlaEgoVehicleStatus vehicle_status);
    float getSlopeForce(CarlaEgoVehicleInfo vehicle_info, CarlaEgoVehicleStatus vehicle_status);
    float getVehicleMaxSteeringAngle(CarlaEgoVehicleInfo vehicle_info);
    float getVehicleMaxSpeed();
    float getVehicleMaxAcceleration();
    float getVehicleMaxDeceleration();
};
#endif  // CARLA_ACKERMANN_CONTROLLER_HPP_