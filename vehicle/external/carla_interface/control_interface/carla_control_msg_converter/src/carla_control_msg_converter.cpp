// AckermannControlCommand to AckermannControlDrive
// cf) carla_control_interface의 carla_ackermann_controller는 ros-bridge의 ackermanncontrol 패키지를 쓰지 않고,
//     바로 AckermannControlCommand형식으로부터 CarlaEgoVehicleControl 메세지 만드는 패키지임. 
//     ==> 이것도 나중에 수정해서 사용해보고 싶음. (두 노드 쓰는 것보다 이게 편리하니까? && cpp니까?)

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>


using autoware_auto_control_msgs::msg::AckermannControlCommand;
using ackermann_msgs::msg::AckermannDrive;


class CarlaControlMsgConverter : public rclcpp::Node
{
public:
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_aw_control_;
  rclcpp::Publisher<AckermannDrive>::SharedPtr pub_carla_control_;

  explicit CarlaControlMsgConverter(const rclcpp::NodeOptions & node_options) : Node("CarlaControlMsgConverter", node_options)
  {
    using std::placeholders::_1;

    const std::string role_name = declare_parameter<std::string>("role_name", "ego_vehicle");

    sub_aw_control_ = create_subscription<AckermannControlCommand>(
      "/control/command/control_cmd", rclcpp::QoS{1}, std::bind(&CarlaControlMsgConverter::onControlCmd, this, _1));
    pub_carla_control_ = create_publisher<AckermannDrive>(
      "/carla/" + role_name + "/ackermann_cmd", rclcpp::QoS{10});
  }
  virtual ~CarlaControlMsgConverter() {}

private:
  void onControlCmd(const AckermannControlCommand::SharedPtr msg)
  {
    ackermann_msgs::msg::AckermannDrive carla_msg;
    carla_msg.steering_angle = msg->lateral.steering_tire_angle;  // [rad] //msg(Autoware: left(+), carla: left(-))
    carla_msg.steering_angle_velocity = msg->lateral.steering_tire_rotation_rate;  // [rad/s]
    carla_msg.speed = msg->longitudinal.speed; // [m/s]
    carla_msg.acceleration = msg->longitudinal.acceleration; // [m/s^2]
    carla_msg.jerk = msg->longitudinal.jerk; // [m/s^3]
    pub_carla_control_ -> publish(carla_msg);
  }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CarlaControlMsgConverter)
