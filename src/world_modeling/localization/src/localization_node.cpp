#include "localization_node.hpp"

Localization::Localization() : Node("localization") {
  // Declare parameters for input topics
  this->declare_parameter<std::string>("gps_topic", "/carla/gnss");
  this->declare_parameter<std::string>("imu_topic", "/carla/imu");
  this->declare_parameter<std::string>("wheel_odom_topic", "/odom");

  // Declare parameters for output topics
  this->declare_parameter<std::string>("gps_output_topic", "/localization/gps");
  this->declare_parameter<std::string>("imu_output_topic", "/localization/imu");
  this->declare_parameter<std::string>("wheel_odom_output_topic", "/localization/odom");

  // Get parameters
  std::string gps_input_topic = this->get_parameter("gps_topic").as_string();
  std::string imu_input_topic = this->get_parameter("imu_topic").as_string();
  std::string odom_input_topic = this->get_parameter("wheel_odom_topic").as_string();

  std::string gps_output_topic = this->get_parameter("gps_output_topic").as_string();
  std::string imu_output_topic = this->get_parameter("imu_output_topic").as_string();
  std::string odom_output_topic = this->get_parameter("wheel_odom_output_topic").as_string();

  // keep last 10 messages
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Create subscribers
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_input_topic, std::bind(&Localization::gps_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_input_topic, qos, std::bind(&Localization::imu_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_input_topic, qos, std::bind(&Localization::odom_callback, this, std::placeholders::_1));

  // Create publishers
  gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_output_topic, qos);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_output_topic, qos);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_output_topic, qos);

  RCLCPP_INFO(this->get_logger(), "Localization node initialized and ready.");
}

// GPS callback
void Localization::gps_callback(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg) {
  // log the message

  gps_pub_->publish(*gps_msg);
}

// IMU callback
void Localization::imu_callback(sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  // log the message

  imu_pub_->publish(*imu_msg);
}

// Wheel odometry callback
void Localization::odom_callback(nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  // log the message

  odom_pub_->publish(*odom_msg);
}

// Main entry point
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Localization>());
  rclcpp::shutdown();
  return 0;
}
