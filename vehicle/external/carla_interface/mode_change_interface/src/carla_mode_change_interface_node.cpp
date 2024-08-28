#include "carla_mode_change_interface/carla_mode_change_interface.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ModeChangeInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
