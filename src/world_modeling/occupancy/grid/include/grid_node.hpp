#ifndef WORLD_MODELING_OCCUPANCY_GRID_NODE_HPP_
#define WORLD_MODELING_OCCUPANCY_GRID_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
// #include <ros/ros.h>
// #include "sample_msgs/msg/pointcloud2.hpp"
// #include "sample_msgs/msg/occupancygrid.hpp"

#include "grid.hpp"

/**
 * Implementation of a ROS2 node that converts lidar messages to occupacygrid
 * messages.
 *
 * Listens to the "LIDAR_TOP" topic and filters out data with invalid fields
 * and odd timestamps. Once the node collects BUFFER_CAPACITY messages it packs
 * the processed messages into an array and publishes it to the "filtered" topic.
 */
class GridNode : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Grid node constructor.
   */
  GridNode();

private:
  /**
   * A ROS2 subscription node callback used to process raw data from the
   * "unfiltered" topic and publish to the "filtered" topic.
   *
   * @param msg a raw message from the "LIDAR_TOP" topic
   */
  void grid_sub_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void grid_publish(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // ROS2 subscriber listening to the LIDAR_TOP topic.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr grid_sub_;

  // ROS2 publisher sending processed messages to the occupancygrid topic.
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

  // Object that handles data processing and validation.
  world_modeling::occupancy::Grid grid_;
};

#endif  // GRID_NODE_HPP_
