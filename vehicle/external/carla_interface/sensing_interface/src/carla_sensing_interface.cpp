#include "carla_sensing_interface/carla_sensing_interface.hpp"

SensingInterface::SensingInterface() : Node("sensing_interface")
{
  // imu
  imu_id_ = declare_parameter("imu_tf", "ego_vehicle/imu");
  input_imu_ = declare_parameter("input_imu", "/carla/ego_vehicle/imu");
  output_imu_ = declare_parameter("output_imu", "/sim/imu");
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    input_imu_, rclcpp::QoS{100},
    std::bind(&SensingInterface::callbackCarlaImu, this, std::placeholders::_1));

  universe_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    output_imu_, rclcpp::QoS{100});

  // gnss
  gnss_id_ = declare_parameter("gnss_tf", "ego_vehicle/gnss");
  input_gnss_ = declare_parameter("input_gnss", "/carla/ego_vehicle/gnss");
  output_gnss_ = declare_parameter("output_gnss", "/sim/gnss");
  gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    input_gnss_, rclcpp::QoS{100},
    std::bind(&SensingInterface::callbackCarlaGnss, this, std::placeholders::_1));

  universe_gnss_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
    output_gnss_, rclcpp::QoS{100});

  // lidar
  lidar_id_ = declare_parameter("lidar_tf", "ego_vehicle/lidar");
  input_lidar_ = declare_parameter("input_lidar", "/carla/ego_vehicle/lidar");
  output_lidar_ = declare_parameter("output_lidar", "/sim/lidar");
  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_lidar_, rclcpp::QoS{100},
    std::bind(&SensingInterface::callbackCarlaLidar, this, std::placeholders::_1));

  universe_lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_lidar_, rclcpp::QoS{100});

  // camera_info
  // camera_info_id_ = declare_parameter("camera_info_tf", "ego_vehicle/rgb_front");
  // input_camera_info_ = declare_parameter("input_camera_info", "/carla/ego_vehicle/rgb_front/camera_info");
  output_camera_info_ = declare_parameter("output_camera_info", "/sim/camera_info");
  // camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
  //   input_camera_info_, rclcpp::QoS{1},
  //   std::bind(&SensingInterface::callbackCarlaCameraInfo, this, std::placeholders::_1));

  universe_camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    output_camera_info_, rclcpp::SensorDataQoS().keep_last(1));

  // image
  image_id_ = declare_parameter("image_tf", "ego_vehicle/rgb_front");
  input_image_ = declare_parameter("input_image", "/carla/ego_vehicle/rgb_front/image");
  output_image_ = declare_parameter("output_image", "/sim/image");
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_image_, rclcpp::QoS{1},
    std::bind(&SensingInterface::callbackCarlaImage, this, std::placeholders::_1));

  universe_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
    output_image_, rclcpp::SensorDataQoS().keep_last(1));
}

void SensingInterface::callbackCarlaImu(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (msg->header.frame_id != imu_id_) {
    RCLCPP_WARN(get_logger(), "imu_id is not ego_vehicle.");
  }
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = this->get_clock()->now();
  imu_msg.header.frame_id = "imu";
  
  imu_msg.orientation = msg->orientation;
  imu_msg.orientation_covariance = msg->orientation_covariance;
  imu_msg.angular_velocity = msg->angular_velocity;
  imu_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu_msg.linear_acceleration = msg->linear_acceleration;
  imu_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;

  universe_imu_pub_->publish(imu_msg);
}

void SensingInterface::callbackCarlaGnss(
  const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->header.frame_id != gnss_id_) {
    RCLCPP_WARN(get_logger(), "gnss_id is not ego_vehicle.");
  }
  sensor_msgs::msg::NavSatFix gnss_msg;
  gnss_msg.header.stamp = this->get_clock()->now();
  gnss_msg.header.frame_id = "gnss";
  
  gnss_msg.status = msg->status;
  gnss_msg.latitude = msg->latitude;
  gnss_msg.longitude = msg->longitude;
  gnss_msg.altitude = msg->altitude;
  gnss_msg.position_covariance = msg->position_covariance;
  gnss_msg.position_covariance_type = msg->position_covariance_type;

  universe_gnss_pub_->publish(gnss_msg);
}

void SensingInterface::callbackCarlaLidar(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (msg->header.frame_id != lidar_id_) {
    RCLCPP_WARN(get_logger(), "lidar_id is not ego_vehicle.");
  }
  sensor_msgs::msg::PointCloud2 lidar_msg;
  lidar_msg.header.stamp = this->get_clock()->now();
  lidar_msg.header.frame_id = "lidar";
  
  lidar_msg.height = msg->height;
  lidar_msg.width = msg->width;
  lidar_msg.fields = msg->fields;
  lidar_msg.is_bigendian = msg->is_bigendian;
  lidar_msg.point_step = msg->point_step;
  lidar_msg.row_step = msg->row_step;
  lidar_msg.data = msg->data;
  lidar_msg.is_dense = msg->is_dense;

  universe_lidar_pub_->publish(lidar_msg);
}


// void SensingInterface::callbackCarlaCameraInfo(
//   const sensor_msgs::msg::CameraInfo::SharedPtr msg)
// {
//   if (msg->header.frame_id != camera_info_id_) {
//     RCLCPP_WARN(get_logger(), "camera_info_id is not ego_vehicle.");
//   }
//   sensor_msgs::msg::CameraInfo camera_info_msg;
//   camera_info_msg.header.stamp = this->get_clock()->now();
//   camera_info_msg.header.frame_id = "camera0/camera_optical_link";
  
//   camera_info_msg.height = msg->height;
//   camera_info_msg.width = msg->width;
//   camera_info_msg.distortion_model = msg->distortion_model;camera_info_msg
//   camera_info_msg.d = msg->d;
//   camera_info_msg.k = msg->k;
//   camera_info_msg.r = msg->r;
//   camera_info_msg.p = msg->p;
//   camera_info_msg.binning_x = msg->binning_x;
//   camera_info_msg.binning_y = msg->binning_y;
//   camera_info_msg.roi = msg->roi;

//   universe_camera_info_pub_->publish(camera_info_msg);
// }

void SensingInterface::callbackCarlaImage(
  const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg->header.frame_id != image_id_) {
    RCLCPP_WARN(get_logger(), "image_id is not ego_vehicle.");
  }
  sensor_msgs::msg::Image image_msg;
  image_msg.header.stamp = this->get_clock()->now();
  image_msg.header.frame_id = "camera0/camera_optical_link";
  
  image_msg.height = msg->height;
  image_msg.width = msg->width;
  image_msg.encoding = msg->encoding;
  image_msg.is_bigendian = msg->is_bigendian;
  image_msg.step = msg->step;
  image_msg.data = msg->data;

  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header.stamp = image_msg.header.stamp;
  camera_info_msg.header.frame_id = "camera0/camera_optical_link";

  camera_info_msg.height = msg->height;
  camera_info_msg.width = msg->width;
  camera_info_msg.distortion_model = "plumb_bob";

  camera_info_msg.d.push_back(0.0);
  camera_info_msg.d.push_back(0.0);
  camera_info_msg.d.push_back(0.0);
  camera_info_msg.d.push_back(0.0);
  camera_info_msg.d.push_back(0.0);

  camera_info_msg.k[0] = 400.00000000000006;
  camera_info_msg.k[1] = 0.0;
  camera_info_msg.k[2] = 400.0;
  camera_info_msg.k[3] = 0.0;
  camera_info_msg.k[4] = 400.00000000000006;
  camera_info_msg.k[5] = 300.0;
  camera_info_msg.k[6] = 0.0;
  camera_info_msg.k[7] = 0.0;
  camera_info_msg.k[8] = 1.0;

  camera_info_msg.r[0] = 1.0;
  camera_info_msg.r[1] = 0.0;
  camera_info_msg.r[2] = 0.0;
  camera_info_msg.r[3] = 0.0;
  camera_info_msg.r[4] = 1.0;
  camera_info_msg.r[5] = 0.0;
  camera_info_msg.r[6] = 0.0;
  camera_info_msg.r[7] = 0.0;
  camera_info_msg.r[8] = 1.0;

  camera_info_msg.p[0] = 400.00000000000006;
  camera_info_msg.p[1] = 0.0;
  camera_info_msg.p[2] = 400.0;
  camera_info_msg.p[3] = 0.0;
  camera_info_msg.p[4] = 0.0;
  camera_info_msg.p[5] = 400.00000000000006;
  camera_info_msg.p[6] = 300.0;
  camera_info_msg.p[7] = 0.0;
  camera_info_msg.p[8] = 0.0;
  camera_info_msg.p[9] = 0.0;
  camera_info_msg.p[10] = 1.0;
  camera_info_msg.p[11] = 0.0;

  camera_info_msg.binning_x = 0;
  camera_info_msg.binning_y = 0;

  camera_info_msg.roi.x_offset = 0;
  camera_info_msg.roi.y_offset = 0;
  camera_info_msg.roi.height = 0;
  camera_info_msg.roi.width = 0;
  camera_info_msg.roi.do_rectify = false;

  universe_image_pub_->publish(image_msg);
  universe_camera_info_pub_->publish(camera_info_msg);
  
}