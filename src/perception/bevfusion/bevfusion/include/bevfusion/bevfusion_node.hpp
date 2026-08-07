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

// clang-format off
// C system headers (.h)
// C++ system headers
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
// Other
#include <deep_msgs/msg/multi_camera_info.hpp>
#include <deep_msgs/msg/multi_detection2_d_array.hpp>
#include <deep_msgs/msg/multi_image_compressed.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/image_marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "bevfusion/bevfusion_core.hpp"
// clang-format on

namespace wato::perception::bevfusion
{

/**
 * @brief ROS 2 lifecycle composable node for BEVFusion
 */
class BEVFusionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  BEVFusionNode() = delete;

  /**
   * @brief Construct the node and declare parameters.
   * @param options Node options for remapping, parameters, etc.
   */
  explicit BEVFusionNode(const rclcpp::NodeOptions & options);

  // Default topic names (matches the remap "from" names in the launch file)
  static constexpr auto kCameraInfoTopic = "camera_info";
  static constexpr auto kLidarTopic = "lidar";
  static constexpr auto kMultiImageTopic = "multi_image";
  static constexpr auto kOutputDetectionsTopic = "output_detections";
  static constexpr auto kOutputMarkersTopic = "output_markers";

  /**
   * @brief Lifecycle: configure parameters, core, QoS, and diagnostics.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS on configuration
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle: create sub/pub, activate publisher, start diagnostics.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS on activation
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle: deactivate publisher and stop processing.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle: destroy pubs/subs and diagnostics, release core.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle: final shutdown and cleanup.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  /**
   * @brief Declare and populate core parameters from the parameter server.
   */
  void declareParameters();

  /**
   * TODO(bevfusion_team) - not sure if we need multiimagecompressed or something else.
   * @brief Main processing callback for synced camera and LiDAR data.
   * @param multi_image_msg MultiImageCompressed containing compressed images from multiple cameras
   * @param point_cloud_msg PointCloud2 containing point cloud data
   */
  void syncedCallback(
    const deep_msgs::msg::MultiImageCompressed::ConstSharedPtr & multi_image_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & point_cloud_msg);

  /**
   * @brief Camera info callback to initialize calibration.
   * @param multi_camera_info_msg MultiCameraInfo containing camera calibration data
   */
  void multiCameraInfoCallback(const deep_msgs::msg::MultiCameraInfo::ConstSharedPtr & multi_camera_info_msg);

  /**
   * @brief Computes calibration matrices from camera info and TF extrinsics, then updates the core CUDA pipeline.
   * @details Called during node activation (`on_activate()`) once camera info and TF transforms are available.
   *          Can also be invoked by `multiCameraInfoCallback()` if dynamic calibration updates are required.
   */
  void computeCalibrationMatrices();

  /**
   * @brief Log total processed messages and average processing time.
   */
  void logStatistics() const;

  /**
   * @brief Build subscriber QoS from reliability/depth parameters.
   * @param reliability Reliability policy ("reliable" or "best_effort")
   * @param depth Queue depth
   * @return Configured QoS profile
   */
  rclcpp::QoS createSubscriberQoS(const std::string & reliability, int depth);

  /**
   * @brief Build publisher QoS from reliability/durability/depth parameters.
   * @param reliability Reliability policy ("reliable" or "best_effort")
   * @param durability Durability policy ("transient_local" or "volatile")
   * @param depth Queue depth
   * @return Configured QoS profile
   */
  rclcpp::QoS createPublisherQoS(const std::string & reliability, const std::string & durability, int depth);

  /**
   * @brief Diagnostic callback for node health/performance.
   * @param stat Diagnostic status wrapper to populate
   */
  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Update counters/timers after processing.
   * @param time_taken Processing time in milliseconds
   */
  void updateStatistics(double time_taken);

  /**
   * @brief Tick topic diagnostics and refresh updater.
   * @param timestamp Timestamp from the processed detection array
   */
  void updateDiagnostics(const std_msgs::msg::Header::_stamp_type & timestamp);

  // Aliases
  using MultiImageMsg = deep_msgs::msg::MultiImageCompressed;
  using MultiCameraInfoMsg = deep_msgs::msg::MultiCameraInfo;
  using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
  using ImageSub = message_filters::Subscriber<MultiImageMsg, rclcpp_lifecycle::LifecycleNode>;
  using LidarSub = message_filters::Subscriber<PointCloud2Msg, rclcpp_lifecycle::LifecycleNode>;
  using SyncPolicy = message_filters::sync_policies::
    ApproximateTime<deep_msgs::msg::MultiImageCompressed, sensor_msgs::msg::PointCloud2>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  /**
   * @brief Decompress a CompressedImage to cv::Mat in BGR format.
   * @param compressed_img The compressed image message
   * @return Decompressed BGR cv::Mat (empty on error)
   */
  cv::Mat decompressImage(const sensor_msgs::msg::CompressedImage & compressed_img) const;

  /**
   * @brief Process a PointCloud2 message and extract LiDAR points.
   * @param lidar_msg The PointCloud2 message
   * @param lidar_data Output vector to flatten andstore extracted LiDAR points
   */
  void processLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg, std::vector<float> & lidar_data);

  /**
   * @brief Convert a BoundingBox to a Detection3D message.
   * @param bbox The BoundingBox to convert
   * @param header The header to use for the Detection3D message
   * @return Detection3D message
   */
  vision_msgs::msg::Detection3D toDetection3D(const BoundingBox & bbox, const std_msgs::msg::Header & header) const;

  /**
   * @brief Convert a BoundingBox to a Marker message.
   * @param bbox The BoundingBox to convert
   * @param header The header to use for the Marker message
   * @param marker_id The ID to assign to the Marker message
   * @return Marker message
   */
  visualization_msgs::msg::Marker toMarker(
    const BoundingBox & bbox, const std_msgs::msg::Header & header, int marker_id) const;

  // Core logic
  std::unique_ptr<BEVFusionCore> core_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Camera info
  rclcpp::Subscription<MultiCameraInfoMsg>::SharedPtr multi_camera_info_sub_;
  MultiCameraInfoMsg::ConstSharedPtr cached_multi_camera_info_;

  // Calibration
  std::atomic<bool> calibration_initialized_{false};

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Subscribers
  std::shared_ptr<LidarSub> lidar_sub_;
  std::shared_ptr<ImageSub> multi_image_sub_;
  // TODO(bevfusion_team) - maybe we should use /camera_pano_nn/image_rect instead for example. Not sure.

  // QoS profiles
  rclcpp::QoS subscriber_qos_;
  rclcpp::QoS publisher_qos_;

  // Synchronization
  std::shared_ptr<Synchronizer> sync_;
  int sync_queue_size_{10};
  double sync_max_time_diff_ms_{200.0};
  double sync_max_time_diff_sec_;
  std::mutex camera_info_mutex_;

  // Statistics
  std::atomic<uint64_t> total_processed_{0};
  std::atomic<double> total_processing_time_ms_{0.0};
  std::atomic<double> last_processing_time_ms_{0.0};
  std::chrono::steady_clock::time_point last_stats_log_time_;
  static constexpr std::chrono::seconds kStatsLogInterval{30};

  // Statistics
  std::atomic<uint64_t> multi_image_msg_count_{0};
  std::atomic<uint64_t> lidar_msg_count_{0};
  std::atomic<uint64_t> synced_msg_count_{0};

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> pub_diagnostic_;
  double min_freq_{0.0};
  double max_freq_{100.0};
};

}  // namespace wato::perception::bevfusion
