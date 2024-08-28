// code for testing carla_ackermann_controller
// this code publishes step control_cmd input to carla_ackermann_controller in ackermann_control_command format

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

using autoware_auto_control_msgs::msg::AckermannControlCommand;

class ControlCmdPublisher : public rclcpp::Node
{
private:
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_ackermann_control_cmd_;
  rclcpp::TimerBase::SharedPtr timer_control_;
  AckermannControlCommand control_cmd_msg_;
  double max_accel_, ctrl_period_, speed_, accel_;
  int duration_, accel_step_, dt_;

  void callbackTimer()
  {
    if(dt_ != -1){
      int step_duration = (duration_-1) / accel_step_;
      for(int i=1; i<=accel_step_; ++i){
        if(dt_ == step_duration * i){
          accel_ = max_accel_ / accel_step_ * i;
          break;
        }
      }
    }
    // calc speed
    // speed_ += accel_ * ctrl_period_;
    // publish
    control_cmd_msg_.stamp = this->now();
    control_cmd_msg_.lateral.steering_tire_angle = 0.0;
    control_cmd_msg_.lateral.steering_tire_rotation_rate = 0.0;
    control_cmd_msg_.longitudinal.speed = speed_;
    control_cmd_msg_.longitudinal.acceleration = accel_;
    
    dt_ = (dt_ + 1) % duration_;
    pub_ackermann_control_cmd_ -> publish(control_cmd_msg_);
  }

public:
  explicit ControlCmdPublisher(const rclcpp::NodeOptions & node_options) 
  : Node("ControlCmdPublisher", node_options),
    max_accel_(declare_parameter<double>("max_accel", 1.0)),
    ctrl_period_(declare_parameter<double>("ctrl_period", 0.03)),
    speed_(declare_parameter<double>("speed", 3.0)),
    accel_(0.0),
    duration_(declare_parameter<int>("duration", 30)),
    accel_step_(declare_parameter<int>("accel_step", 3)),
    dt_(-1)
  {
    pub_ackermann_control_cmd_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd_jw", rclcpp::QoS{10});
    const auto period_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(ctrl_period_));
    timer_control_ = rclcpp::create_timer(this, get_clock(), period_ns_, std::bind(&ControlCmdPublisher::callbackTimer, this));
  }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ControlCmdPublisher)