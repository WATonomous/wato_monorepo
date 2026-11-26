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

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <deep_msgs/msg/multi_camera_info.hpp>
#include <deep_msgs/msg/multi_image.hpp>
#include <deep_msgs/msg/multi_image_compressed.hpp>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet_msgs/msg/map_visualization.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace semvecnet
{
class SemVecNet;
namespace postprocessing
{
struct Polyline;
}  // namespace postprocessing
}  // namespace semvecnet

namespace wato::perception::lanelets
{

class LaneletsCore;

/// Synchronizes rectified multi-camera frames and LiDAR for SemVecNet.
class LaneletsNode : public rclcpp::Node
{
public:
  explicit LaneletsNode(const rclcpp::NodeOptions & options);
  ~LaneletsNode() override;

private:
  using MultiImage = deep_msgs::msg::MultiImage;
  using MultiImageCompressed = deep_msgs::msg::MultiImageCompressed;
  using MultiCameraInfo = deep_msgs::msg::MultiCameraInfo;
  using PointCloud = sensor_msgs::msg::PointCloud2;
  using RawSyncPolicy = message_filters::sync_policies::ApproximateTime<MultiImage, PointCloud>;
  using CompressedSyncPolicy = message_filters::sync_policies::ApproximateTime<MultiImageCompressed, PointCloud>;

  void syncedRawCallback(const MultiImage::ConstSharedPtr & images, const PointCloud::ConstSharedPtr & cloud);
  void syncedCompressedCallback(
    const MultiImageCompressed::ConstSharedPtr & images, const PointCloud::ConstSharedPtr & cloud);
  void processFrame(std::vector<cv::Mat> & rgb_images, const PointCloud & cloud);
  // std::vector<Polyline> is semvecnet::Polylines. Return geometry in map_frame_.
  lanelet::Lanelets generateLanelets(
    const std::vector<semvecnet::postprocessing::Polyline> & polylines, const std::array<float, 16> & ego_to_map);
  void publishLanelets(const lanelet::Lanelets & lanelets, const rclcpp::Time & stamp);

  std::unique_ptr<LaneletsCore> core_;
  std::unique_ptr<semvecnet::SemVecNet> semvecnet_;
  std::vector<std::string> camera_names_;
  // Returns false if this frame arrives sooner than 1/processing_rate_hz after the last accepted one.
  bool acceptFrame(const builtin_interfaces::msg::Time &stamp);
  double min_period_s_{0.0};
  bool has_last_stamp_{false};
  rclcpp::Time last_accepted_stamp_{0, 0, RCL_ROS_TIME};
  std::string map_frame_;
  std::string ego_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  // All sensor callbacks use the default mutually exclusive callback group.
  MultiCameraInfo::ConstSharedPtr cached_camera_info_;
  rclcpp::Subscription<MultiCameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<lanelet_msgs::msg::MapVisualization>::SharedPtr lanelets_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr polyline_markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub_;
  message_filters::Subscriber<PointCloud> lidar_sub_;
  message_filters::Subscriber<MultiImage> raw_sub_;
  message_filters::Subscriber<MultiImageCompressed> compressed_sub_;
  std::unique_ptr<message_filters::Synchronizer<RawSyncPolicy>> raw_sync_;
  std::unique_ptr<message_filters::Synchronizer<CompressedSyncPolicy>> compressed_sync_;
};

}  // namespace wato::perception::lanelets
