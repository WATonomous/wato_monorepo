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

#pragma once

#include <mutex>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace wato::perception::lanelets
{

/**
 * @brief Skeleton node that receives camera and LiDAR data for lanelet generation.
 *
 * Subscriptions (remap in the launch file):
 *   input_image        sensor_msgs/CompressedImage (use_compressed_images: true) or sensor_msgs/Image
 *   input_camera_info  sensor_msgs/CameraInfo
 *   input_cloud        sensor_msgs/PointCloud2
 */
class LaneletsNode : public rclcpp::Node
{
public:
  explicit LaneletsNode(const rclcpp::NodeOptions & options);

  static constexpr auto kImageTopic = "input_image";
  static constexpr auto kCameraInfoTopic = "input_camera_info";
  static constexpr auto kCloudTopic = "input_cloud";

private:
  void onCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);
  void onRawImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  /// Stores the decoded image so the LiDAR callback can pair it with the cloud.
  void storeImage(const std_msgs::msg::Header & header, const cv::Mat & bgr);

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::mutex mutex_;
  cv::Mat latest_image_;  ///< BGR8
  std_msgs::msg::Header latest_image_header_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
};

}  // namespace wato::perception::lanelets
