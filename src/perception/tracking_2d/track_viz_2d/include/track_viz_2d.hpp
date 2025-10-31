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

#ifndef TRACK_VIZ_2D_HPP
#define TRACK_VIZ_2D_HPP

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tracking_2d_msgs/msg/tracking2_d_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class track_viz_2d : public rclcpp::Node
{
public:
  track_viz_2d();

private:
  // Callback functions
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void tracksCallback(const tracking_2d_msgs::msg::Tracking2DArray::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  // Helper functions
  cv::Scalar colorLookup(std::string color);
  void drawDets(cv::Mat & image, const std::vector<vision_msgs::msg::Detection2D> & dets, const cv::Scalar & color);
  void drawTracks(
    cv::Mat & image, const std::vector<tracking_2d_msgs::msg::Tracking2D> & trks, const cv::Scalar & color);
  void tryDraw(cv::Mat & decoded_img, const tracking_2d_msgs::msg::Tracking2DArray::SharedPtr latest_trks_);

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr dets_sub_;
  rclcpp::Subscription<tracking_2d_msgs::msg::Tracking2DArray>::SharedPtr trks_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  // Parameters
  void initializeParams();
  std::string detections_topic_;
  std::string track_topic_;
  std::string image_sub_topic_;
  std::string image_pub_topic_;
  std::string camera_frame_;
  std::string color_dets_;
  std::string color_trks_;
  int bbox_line_width_;

  std::unordered_map<std::string, cv::Scalar> color_map_;
  cv::Scalar default_color_;

  sensor_msgs::msg::CompressedImage::SharedPtr latest_image_;
  vision_msgs::msg::Detection2DArray::SharedPtr latest_dets_;
};

#endif
