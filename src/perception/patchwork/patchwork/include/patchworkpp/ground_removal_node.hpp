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

#include <stdint.h>

#include <Eigen/Core>

// clang-format off
#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "patchworkpp/ground_removal_core.hpp"
// clang-format on

namespace wato::perception::patchworkpp
{

/**
 * @brief ROS 2 lifecycle node for Patchwork++ ground removal.
 *
 * Subscribes to a point cloud, segments ground/non-ground via Patchwork++,
 * publishes both outputs, and reports diagnostics/statistics.
 */
class GroundRemovalNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  GroundRemovalNode() = delete;

  /**
   * @brief Construct the node and declare parameters/QoS settings.
   * @param options Node options for remapping, parameters, etc.
   */
  explicit GroundRemovalNode(const rclcpp::NodeOptions & options);

  static constexpr auto kCloudTopic = "input_cloud";  ///< Input point cloud topic name
  static constexpr auto kGroundTopic = "ground_cloud";  ///< Output ground points topic name
  static constexpr auto kNonGroundTopic = "non_ground_cloud";  ///< Output non-ground points topic name

  /**
   * @brief Lifecycle: configure parameters/core/QoS/diagnostics.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS on configuration
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle: create sub/pub, activate publishers, start diagnostics.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS on activation
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle: deactivate publishers and stop processing.
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
   * @brief Populate Patchwork++ params from declared parameters.
   * @param params Output parameter struct
   */
  void declareParameters(patchwork::Params & params);

  /**
   * @brief Point cloud subscription callback: process and publish segments.
   * @param msg Incoming point cloud message
   */
  void removeGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  /**
   * @brief Publish ground and non-ground clouds with the given header.
   * @param ground_points Ground points as Eigen matrix
   * @param nonground_points Non-ground points as Eigen matrix
   * @param header Header to copy to output messages
   */
  void publishSegments(
    const Eigen::MatrixX3f & ground_points,
    const Eigen::MatrixX3f & nonground_points,
    const std_msgs::msg::Header & header);

  /**
   * @brief Log total processed clouds and average processing time.
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
   * @brief Remove NaN/Inf points from a cloud.
   * @param cloud Input point cloud
   * @return Filtered cloud with only finite points (may be empty)
   */
  Eigen::MatrixX3f filterInvalidPoints(const Eigen::MatrixX3f & cloud);

  /**
   * @brief Update counters/timers after processing.
   * @param time_taken Processing time in milliseconds
   */
  void updateStatistics(double time_taken);

  /**
   * @brief Tick topic diagnostics and refresh updater.
   * @param timestamp Timestamp from the processed point cloud
   */
  void updateDiagnostics(const std_msgs::msg::Header::_stamp_type & timestamp);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;

  std::unique_ptr<GroundRemovalCore> core_;

  rclcpp::QoS subscriber_qos_;  ///< QoS profile for point cloud subscriber (configured in on_configure)
  rclcpp::QoS publisher_qos_;  ///< QoS profile for publishers (configured in on_configure)

  std::atomic<uint64_t> total_processed_{0};  ///< Total number of point clouds processed
  std::atomic<double> total_processing_time_ms_{0.0};  ///< Cumulative processing time in milliseconds
  std::atomic<double> last_processing_time_ms_{0.0};  ///< Processing time for last cloud in milliseconds
  std::chrono::steady_clock::time_point last_stats_log_time_;  ///< Last time statistics were logged
  static constexpr std::chrono::seconds kStatsLogInterval{30};  ///< Statistics logging interval

  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;  ///< Diagnostic updater for node health
  std::unique_ptr<diagnostic_updater::TopicDiagnostic>
    ground_pub_diagnostic_;  ///< Topic diagnostics for ground publisher
  std::unique_ptr<diagnostic_updater::TopicDiagnostic>
    nonground_pub_diagnostic_;  ///< Topic diagnostics for non-ground publisher
  double min_freq_{0.0};  ///< Minimum expected publishing frequency (Hz)
  double max_freq_{100.0};  ///< Maximum expected publishing frequency (Hz)
};

}  // namespace wato::perception::patchworkpp
