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

#include "attribute_assigner/attribute_assigner_core.hpp"
// clang-format on

namespace wato::perception::attribute_assigner
{

/**
 * @brief ROS 2 lifecycle composable node for assigning semantic attributes to 2D detections.
 *
 * Synchronizes MultiImage (from camera_sync) and Detection2DArray topics.
 * Uses detection-driven processing: when synchronized messages arrive, looks up
 * relevant images by detection frame_id, decompresses only needed images once,
 * crops to detection bounding boxes, and enriches traffic light and car detections
 * with attribute hypotheses (traffic light state, car behavior).
 *
 * Traffic light attributes: green, yellow, red
 * Car behavior attributes: turning left, turning right, braking, hazard lights
 *
 * Each attribute is appended as an additional ObjectHypothesisWithPose entry in
 * the detection's results array using a namespaced class_id (e.g., "state:green",
 * "behavior:braking").
 */
class AttributeAssignerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  AttributeAssignerNode() = delete;

  /**
   * @brief Construct the node and declare parameters.
   * @param options Node options for remapping, parameters, etc.
   */
  explicit AttributeAssignerNode(const rclcpp::NodeOptions & options);

  static constexpr auto kMultiImageTopic = "input_multi_image";  ///< Input MultiImage topic
  static constexpr auto kMultiCameraInfoTopic = "input_multi_camera_info";  ///< Input MultiCameraInfo topic
  static constexpr auto kInputTopic = "input_detections";  ///< Input detections topic name
  static constexpr auto kOutputTopic = "output_detections";  ///< Output enriched detections topic name
  static constexpr auto kImageMarkersTopic = "/perception/enriched_detection_markers";  ///< Image markers topic
  static constexpr auto kDetections3DTopic = "/perception/detections_3d";  ///< 3D detections topic
  static constexpr auto kDetections3DMarkersTopic =
    "/perception/detections_3d_markers";  ///< 3D detection markers topic

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
   * @param params Output parameter struct
   */
  void declareParameters(Params & params);

  /**
   * @brief Synced callback: MultiImage + detections via ApproximateTime sync.
   * @param multi_image_msg MultiImageCompressed containing compressed images from multiple cameras
   * @param detections_msg MultiDetection2DArray with per-camera Detection2DArrays
   */
  void syncedCallback(
    const deep_msgs::msg::MultiImageCompressed::ConstSharedPtr & multi_image_msg,
    const deep_msgs::msg::MultiDetection2DArray::ConstSharedPtr & detections_msg);

  /**
   * @brief Callback for multi-camera-info messages (cached, not synced)
   * @param msg Multi-camera-info message
   */
  void multiCameraInfoCallback(const deep_msgs::msg::MultiCameraInfo::ConstSharedPtr & msg);

  /**
   * @brief Decompress a CompressedImage to cv::Mat in BGR format.
   * @param compressed_img The compressed image message
   * @return Decompressed BGR cv::Mat (empty on error)
   */
  cv::Mat decompressImage(const sensor_msgs::msg::CompressedImage & compressed_img) const;

  /**
   * @brief Create image markers for detection visualization.
   * @param detections Detection array to visualize
   * @param decompressed_images Map of frame_id to decompressed images
   * @param stamp Timestamp for the markers
   * @return Vector of ImageMarker messages with visualization markers
   */
  std::vector<visualization_msgs::msg::ImageMarker> createDetectionMarkers(
    const deep_msgs::msg::MultiDetection2DArray & detections,
    const std::unordered_map<std::string, cv::Mat> & decompressed_images,
    const builtin_interfaces::msg::Time & stamp) const;

  /**
   * @brief Create 3D detections for traffic lights and cars using camera intrinsics and TF.
   * @param detections 2D detection array
   * @param camera_infos Map of frame_id to CameraInfo
   * @param stamp Timestamp for the 3D detections
   * @return Detection3DArray with 3D positions for traffic lights and cars
   */
  vision_msgs::msg::Detection3DArray create3DDetections(
    const deep_msgs::msg::MultiDetection2DArray & detections,
    const std::unordered_map<std::string, sensor_msgs::msg::CameraInfo> & camera_infos,
    const builtin_interfaces::msg::Time & stamp) const;

  /**
   * @brief Convert 3D detections to a MarkerArray for Foxglove visualization.
   * @param detections_3d 3D detection array to convert
   * @return MarkerArray with cube markers colored by type/state
   */
  visualization_msgs::msg::MarkerArray create3DMarkers(const vision_msgs::msg::Detection3DArray & detections_3d) const;

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

  using MultiImageMsg = deep_msgs::msg::MultiImageCompressed;
  using MultiCameraInfoMsg = deep_msgs::msg::MultiCameraInfo;
  using DetectionsMsg = deep_msgs::msg::MultiDetection2DArray;

  // ApproximateTime sync for MultiImage + Detections
  using ImageSub = message_filters::Subscriber<MultiImageMsg, rclcpp_lifecycle::LifecycleNode>;
  using DetSub = message_filters::Subscriber<DetectionsMsg, rclcpp_lifecycle::LifecycleNode>;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<MultiImageMsg, DetectionsMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::shared_ptr<ImageSub> multi_image_sub_;
  std::shared_ptr<DetSub> detections_sub_;
  std::shared_ptr<Synchronizer> sync_;

  // CameraInfo is cached separately (changes infrequently)
  rclcpp::Subscription<MultiCameraInfoMsg>::SharedPtr multi_camera_info_sub_;
  std::mutex sync_mutex_;
  MultiCameraInfoMsg::ConstSharedPtr cached_multi_camera_info_;

  // TF2 for camera transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_lifecycle::LifecyclePublisher<deep_msgs::msg::MultiDetection2DArray>::SharedPtr detections_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::ImageMarker>::SharedPtr image_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_3d_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr detections_3d_markers_pub_;

  std::unique_ptr<AttributeAssignerCore> core_;

  rclcpp::QoS subscriber_qos_;  ///< QoS profile for detection subscriber (configured in on_configure)
  rclcpp::QoS publisher_qos_;  ///< QoS profile for publisher (configured in on_configure)

  std::string multi_image_topic_;  ///< Configured multi-image topic name
  std::string multi_camera_info_topic_;  ///< Configured multi-camera-info topic name
  std::string input_detections_topic_;  ///< Configured input detections topic name
  std::string output_detections_topic_;  ///< Configured output detections topic name
  double sync_max_time_diff_sec_;  ///< Maximum time difference for synchronization (seconds)
  std::string target_frame_;  ///< Target frame for 3D detections (e.g., "base_link")
  double traffic_light_assumed_depth_;  ///< Assumed depth for traffic lights in meters
  double car_assumed_depth_;  ///< Assumed depth for cars in meters
  double car_real_width_;  ///< Assumed car width in meters
  double car_real_height_;  ///< Assumed car height in meters
  double car_real_length_;  ///< Assumed car length in meters

  double truck_real_width_;  ///< Assumed truck width in meters
  double truck_real_height_;  ///< Assumed truck height in meters
  double truck_real_length_;  ///< Assumed truck length in meters

  double bus_real_width_;  ///< Assumed bus width in meters
  double bus_real_height_;  ///< Assumed bus height in meters
  double bus_real_length_;  ///< Assumed bus length in meters

  bool enable_image_markers_{true};  ///< Enable 2D image marker overlays
  bool enable_3d_markers_{true};  ///< Enable 3D visualization markers

  std::atomic<uint64_t> multi_image_msg_count_{0};  ///< Count of multi-image messages received
  std::atomic<uint64_t> detections_msg_count_{0};  ///< Count of detection messages received
  std::atomic<uint64_t> synced_msg_count_{0};  ///< Count of synchronized message pairs

  std::atomic<uint64_t> total_processed_{0};  ///< Total detection arrays processed
  std::atomic<double> total_processing_time_ms_{0.0};  ///< Cumulative processing time (ms)
  std::atomic<double> last_processing_time_ms_{0.0};  ///< Processing time for last array (ms)
  std::chrono::steady_clock::time_point last_stats_log_time_;  ///< Last statistics log time
  static constexpr std::chrono::seconds kStatsLogInterval{30};  ///< Statistics logging interval

  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;  ///< Diagnostic updater
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> pub_diagnostic_;  ///< Topic diagnostics for publisher
  double min_freq_{0.0};  ///< Minimum expected publishing frequency (Hz)
  double max_freq_{100.0};  ///< Maximum expected publishing frequency (Hz)

  int sync_queue_size_{10};  ///< Message filters sync queue size
  double sync_max_time_diff_ms_{200.0};  ///< Max time difference for message sync (ms)
};

}  // namespace wato::perception::attribute_assigner
