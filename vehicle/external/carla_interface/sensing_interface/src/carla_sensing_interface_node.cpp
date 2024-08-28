#include "carla_sensing_interface/carla_sensing_interface.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensingInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
