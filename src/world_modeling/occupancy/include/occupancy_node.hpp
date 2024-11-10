#ifndef OCCUPANCY_NODE_HPP_
#define OCCUPANCY_NODE_HPP_
#define PCL_NO_PRECOMPILE

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "occupancy_core.hpp"

/**
 * Implementation of a ROS2 node that converts 3D PointCloud2 points to 2D by stripping away the z-dimension.
 *
 * Listens to the "nonground_points" topic and removes the z-axis of the 3D points.
 * Once it processes the points, it publishes it to the "costmap" topic.
 */
class OccupancyNode : public rclcpp::Node {
 public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Occupancy node constructor.
   */
  OccupancyNode();

 private:
  // Object that handles data processing and validation.
  OccupancyCore occupancy_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _publisher;

  void subscription_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud);
};

#endif  // OCCUPANCY_NODE_HPP_
