#include <rclcpp/rclcpp.hpp>
// #include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.h>

#include <array>
#include <string>
#include <vector>

class ModeChangeInterface : public rclcpp::Node
{
public:
  ModeChangeInterface();
  ~ModeChangeInterface() = default;
private:
  void callbackCarlaGnss(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_sub_;

  std::string map_id_;
};
