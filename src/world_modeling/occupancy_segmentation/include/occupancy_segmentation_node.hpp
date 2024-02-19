#ifndef OCCUPANCY_SEGMENTATION_NODE_HPP_
#define OCCUPANCY_SEGMENTATION_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "sample_msgs/msg/unfiltered.hpp"
#include "std_msgs/msg/int32.hpp"
// #include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
// #include "image_geometry/pinhole_camera_model.h"


#include "occupancy_segmentation_core.hpp"

/**
 * Implementation of a ROS2 node that generates unfiltered ROS2 messages on a
 * time interval.
 */
class OccupancySegmentationNode: public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * occupancy_segmentation node constructor.
   *
   * @param delay_ms the frequency with which the node produces data.
   */
  explicit OccupancySegmentationNode();

private:

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void cam_back_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void cam_back_left_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void cam_back_right_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void cam_front_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void cam_front_left_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void cam_front_right_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pt_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_back_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_back_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_back_right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_front_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_front_right_sub_;

};

#endif  // OCCUPANCY_SEGMENTATION_NODE_HPP_
