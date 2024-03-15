#ifndef LANE_DETECTION_NODE_HPP
#define LANE_DETECTION_NODE_HPP

#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "lane_detection_msgs/msg/lane_detection.hpp"

#include <opencv2/opencv.hpp>

#include "common_helper_cv.h"
#include "image_processor.h"

class LaneDetectionNode : public rclcpp::Node {
 public:
  LaneDetectionNode();
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<lane_detection_msgs::msg::LaneDetection>::SharedPtr lane_detection_pub_;

  size_t count_;

  // ROS Parameters
  std::string camera_topic_;
  std::string publish_vis_topic_;
  std::string publish_lanes_topic_;
  bool save_images_;
  std::string save_dir_;
  bool publish_source_image_;
};

#endif  // LANE_DETECTION_NODE_HPP