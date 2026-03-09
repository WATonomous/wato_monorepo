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
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
// C++ system headers
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
// Other
#include <deep_msgs/msg/multi_image_compressed.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

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
  static constexpr auto kInputTopic = "input_detections";  ///< Input detections topic name
  static constexpr auto kOutputTopic = "output_detections";  ///< Output enriched detections topic name

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
   * @brief Synced callback: MultiImage + detections. Decompresses only needed images, crops, classifies.
   * @param multi_image_msg MultiImageCompressed containing compressed images from multiple cameras
   * @param detections_msg Detection2DArray with detections indexed by source_id (frame_id)
   */
  void syncedCallback(
    const deep_msgs::msg::MultiImageCompressed::ConstSharedPtr & multi_image_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections_msg);

  /**
   * @brief Decompress a CompressedImage to cv::Mat in BGR format.
   * @param compressed_img The compressed image message
   * @return Decompressed BGR cv::Mat (empty on error)
   */
  cv::Mat decompressImage(const sensor_msgs::msg::CompressedImage & compressed_img) const;

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
  using DetectionsMsg = vision_msgs::msg::Detection2DArray;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<MultiImageMsg, DetectionsMsg>;

  std::unique_ptr<message_filters::Subscriber<MultiImageMsg, rclcpp_lifecycle::LifecycleNode>> multi_image_sub_;
  std::unique_ptr<message_filters::Subscriber<DetectionsMsg, rclcpp_lifecycle::LifecycleNode>> detections_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;

  std::unique_ptr<AttributeAssignerCore> core_;

  rclcpp::QoS subscriber_qos_;  ///< QoS profile for detection subscriber (configured in on_configure)
  rclcpp::QoS publisher_qos_;  ///< QoS profile for publisher (configured in on_configure)

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
};

}  // namespace wato::perception::attribute_assigner
