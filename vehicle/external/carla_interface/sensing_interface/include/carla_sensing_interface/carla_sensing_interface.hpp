#include <rclcpp/rclcpp.hpp>
// #include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.h>

#include <array>
#include <string>
#include <vector>

class SensingInterface : public rclcpp::Node
{
public:
  SensingInterface();
  ~SensingInterface() = default;

private:
  // imu
  void callbackCarlaImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
    imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr
    universe_imu_pub_;

  // gnss
  void callbackCarlaGnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr
    gnss_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr
    universe_gnss_pub_;

  // lidar
  void callbackCarlaLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    universe_lidar_pub_;

  // camera_info 
  // void callbackCarlaCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
  //   camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
    universe_camera_info_pub_;

  // image
  void callbackCarlaImage(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    universe_image_pub_;

  std::string imu_id_;
  std::string gnss_id_;
  std::string lidar_id_;
  // std::string camera_info_id_;s
  std::string image_id_;

  std::string input_imu_;
  std::string input_gnss_;
  std::string input_lidar_;
  // std::string input_camera_info_;
  std::string input_image_;

  std::string output_imu_;
  std::string output_gnss_;
  std::string output_lidar_;
  std::string output_camera_info_;
  std::string output_image_;
};
