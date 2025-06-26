#ifndef WORLD_MODELING_LOCALIZATION
#define WORLD_MODELING_LOCALIZATION


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

class Localization : public rclcpp::Node {
public:
  Localization();

  /**
   * @brief This ROS node is responsible for subscribing to GPS, IMU, and wheel odometry data 
   * coming from Carla first, then the vehicle's onboard systems. The primary purpose of this node
   * is to confirm that sensor data is being received correctly, and to act as an intermediate
   * step where preprocessing can be applied before the data is passed to the robot_localization EKF node.
   */

private:
  void gps_callback(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);
  void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr odom_msg);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publishe`r<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

#endif  // WORLD_MODELING_LOCALIZATION
