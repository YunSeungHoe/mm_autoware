#include "carla_mode_change_interface/carla_mode_change_interface.hpp"

ModeChangeInterface::ModeChangeInterface() : Node("mode_change_interface")
{
  map_id_ = declare_parameter("map_id", "map");
  gnss_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/sensing/gnss/gnss_pose", rclcpp::QoS{100},
    std::bind(&ModeChangeInterface::callbackCarlaGnss, this, std::placeholders::_1));
}


void ModeChangeInterface::callbackCarlaGnss(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg->header.frame_id != map_id_) {
    RCLCPP_WARN(get_logger(), "map_id is not map.");
  }
  if (msg->pose.position.x >= 0 && msg->pose.position.y >= 0)
    std::cout << "Urban mode" << std::endl;
  else
    std::cout << "Highway mode" << std::endl;
}
