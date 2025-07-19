// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LANE_DETECTION_NODE_HPP
#define LANE_DETECTION_NODE_HPP

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "lane_detection/common_helper_cv.h"
#include "lane_detection/image_processor.h"
#include "lane_detection_msgs/msg/lane_detection.hpp"

class LaneDetectionNode : public rclcpp::Node
{
public:
  LaneDetectionNode();
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

private:
  void populate_lane_msg(
    const lane_detection_msgs::msg::LaneDetection & lane_msg, const std::vector<std::vector<float>> & raw_lane_list);
  void save_image(const cv::Mat & image, const std::string & filename);

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
  // Debug will publish the source image with lane detection overlay
  bool debug_node_;
};

#endif  // LANE_DETECTION_NODE_HPP
