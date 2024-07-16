#ifndef WORLD_MODELING_HD_MAP_LOCALIZATION_NODE_HPP_
#define WORLD_MODELING_HD_MAP_LOCALIZATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class LocalizationNode : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Sample node constructor.
   */
  LocalizationNode();

private:
  void sample_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void sample_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // ROS2 subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // ROS2 publishers

  // Object that handles data processing and validation.
  world_modeling::hd_map::Sample sample_;
};

#endif // LOCALIZATION_NODE_HPP_