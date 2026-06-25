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

  // Core logic
  std::unique_ptr<BEVFusionCore> core_;

  // QoS profiles
  rclcpp::QoS subscriber_qos_;
  rclcpp::QoS publisher_qos_;

  // Synchronization parameters
  int sync_queue_size_{10};
  double sync_max_time_diff_ms_{200.0};
  double sync_max_time_diff_sec_;

  // Statistics
  std::atomic<uint64_t> total_processed_{0};
  std::atomic<double> total_processing_time_ms_{0.0};
  std::atomic<double> last_processing_time_ms_{0.0};
  std::chrono::steady_clock::time_point last_stats_log_time_;
  static constexpr std::chrono::seconds kStatsLogInterval{30};

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> pub_diagnostic_;
  double min_freq_{0.0};
  double max_freq_{100.0};
};

}  // namespace wato::perception::bevfusion
