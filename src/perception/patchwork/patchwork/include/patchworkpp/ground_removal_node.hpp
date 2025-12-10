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

#include <atomic>
#include <chrono>
#include <cstdint>

#include <Eigen/Core>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "patchworkpp/ground_removal_core.hpp"

namespace wato::perception::patchworkpp
{

/**
 * @brief Point cloud validation limits
 *
 * Configurable limits for point cloud validation to prevent processing
 * unreasonably large or malformed point clouds. Different limit profiles
 * can be configured for different LiDAR types (typical, high-density, etc.)
 */
struct PointCloudLimits
{
  /**
   * @brief Maximum number of points allowed in a point cloud
   *
   * Point clouds exceeding this limit will be rejected to prevent
   * memory exhaustion and excessive processing time.
   *
   * Typical values:
   * - Standard LiDAR (e.g., Velodyne VLP-16): 100,000 - 500,000 points
   * - High-density LiDAR (e.g., Velodyne VLP-32C): 1,000,000 - 2,000,000 points
   * - Very high-density (e.g., Ouster OS1-128): 2,000,000 - 10,000,000 points
   */
  size_t max_points;

  /**
   * @brief Minimum number of points expected in a valid point cloud
   *
   * Point clouds with fewer points than this will be rejected as likely
   * malformed or empty. Set to 0 to allow empty clouds (not recommended).
   */
  size_t min_points;

  /**
   * @brief Create default limits for typical LiDAR sensors
   */
  static PointCloudLimits defaultLimits()
  {
    return PointCloudLimits{10000000, 0};  // 10M max, 0 min (allow empty)
  }

  /**
   * @brief Create limits for high-density LiDAR sensors
   */
  static PointCloudLimits highDensityLimits()
  {
    return PointCloudLimits{20000000, 0};  // 20M max for very dense sensors
  }

  /**
   * @brief Create limits for standard LiDAR sensors
   */
  static PointCloudLimits standardLimits()
  {
    return PointCloudLimits{5000000, 0};  // 5M max for typical sensors
  }
};

/**
 * @brief ROS 2 lifecycle node for Patchwork++ ground removal algorithm
 *
 * This node processes LiDAR point clouds to segment ground and non-ground points
 * using the Patchwork++ algorithm. It implements a ROS 2 lifecycle node pattern
 * for proper state management and supports configurable parameters, QoS settings,
 * diagnostics, and statistics tracking.
 */
class GroundRemovalNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  GroundRemovalNode() = delete;

  /**
   * @brief Constructs the Patchwork++ ROS 2 lifecycle node
   *
   * Declares all algorithm parameters and QoS settings. Actual initialization
   * of subscribers and publishers occurs in on_configure() and on_activate()
   * lifecycle callbacks respectively.
   *
   * @param options Node options for remapping, parameters, etc.
   */
  explicit GroundRemovalNode(const rclcpp::NodeOptions & options);

  static constexpr auto kCloudTopic = "input_cloud";      ///< Input point cloud topic name
  static constexpr auto kGroundTopic = "ground_cloud";    ///< Output ground points topic name
  static constexpr auto kNonGroundTopic = "non_ground_cloud";  ///< Output non-ground points topic name

  /**
   * @brief Lifecycle callback: Configures the node
   *
   * Initializes GroundRemovalCore with parameters, sets up QoS profiles,
   * and initializes diagnostic updater. Does not create subscribers/publishers
   * yet (done in on_activate).
   *
   * @param previous_state Previous lifecycle state
   * @return SUCCESS if configuration succeeds, FAILURE otherwise
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle callback: Activates the node
   *
   * Creates subscribers and publishers, activates lifecycle publishers,
   * and sets up topic diagnostics. Node starts processing point clouds.
   *
   * @param previous_state Previous lifecycle state
   * @return SUCCESS if activation succeeds, FAILURE otherwise
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle callback: Deactivates the node
   *
   * Stops processing but keeps resources allocated for quick reactivation.
   * Deactivates publishers and resets subscriber.
   *
   * @param previous_state Previous lifecycle state
   * @return SUCCESS
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle callback: Cleans up resources
   *
   * Destroys subscribers, publishers, releases core resources, and cleans up
   * diagnostics. Node can be reconfigured after cleanup.
   *
   * @param previous_state Previous lifecycle state
   * @return SUCCESS
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle callback: Shuts down the node
   *
   * Performs final cleanup and resource release. Node cannot be restarted
   * after shutdown.
   *
   * @param previous_state Previous lifecycle state
   * @return SUCCESS
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  /**
   * @brief Declares and populates Patchwork++ algorithm parameters
   *
   * Retrieves parameter values from the node and writes them into the provided
   * params struct. Parameters include sensor height, iteration counts, thresholds,
   * ranges, and verbosity settings.
   *
   * @param params Output parameter struct to populate
   */
  void declareParameters(patchwork::Params & params);

  /**
   * @brief Subscription callback for incoming PointCloud2 messages
   *
   * Main processing callback that:
   * - Converts PointCloud2 to Eigen matrix
   * - Validates and filters point cloud
   * - Runs ground removal algorithm
   * - Updates statistics and diagnostics
   * - Publishes ground and non-ground segments
   *
   * @param msg Incoming point cloud message
   */
  void removeGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  /**
   * @brief Publishes ground and non-ground point cloud segments
   *
   * Converts Eigen matrices to PointCloud2 messages (XYZ, FLOAT32) while
   * preserving the incoming header (frame, timestamp). Only publishes if
   * publishers are activated and ready.
   *
   * @param ground_points Ground points as Eigen matrix
   * @param nonground_points Non-ground points as Eigen matrix
   * @param header Header to copy to output messages
   */
  void publishSegments(
    const Eigen::MatrixX3f & ground_points,
    const Eigen::MatrixX3f & nonground_points,
    const std_msgs::msg::Header & header);

  /**
   * @brief Logs processing statistics to console
   *
   * Logs total processed clouds and average processing time. Called periodically
   * or on demand for performance monitoring.
   */
  void logStatistics() const;

  /**
   * @brief Creates a QoS profile for the point cloud subscriber
   *
   * Configures reliability and depth based on parameters. Defaults to best_effort
   * reliability for low-latency sensor data processing.
   *
   * @param reliability Reliability policy ("reliable" or "best_effort")
   * @param depth Queue depth for incoming messages
   * @return Configured QoS profile
   */
  rclcpp::QoS createSubscriberQoS(const std::string & reliability, int depth);

  /**
   * @brief Creates a QoS profile for publishers
   *
   * Configures reliability, durability, and depth based on parameters.
   * Defaults to reliable + transient_local for immediate availability to
   * late-joining subscribers (e.g., RViz).
   *
   * @param reliability Reliability policy ("reliable" or "best_effort")
   * @param durability Durability policy ("transient_local" or "volatile")
   * @param depth Queue depth for outgoing messages
   * @return Configured QoS profile
   */
  rclcpp::QoS createPublisherQoS(const std::string & reliability, const std::string & durability, int depth);

  /**
   * @brief Diagnostic callback for monitoring node health and performance
   *
   * Reports processing statistics, latency metrics, and operational status.
   * Sets diagnostic status to OK, WARN, or ERROR based on performance thresholds.
   *
   * @param stat Diagnostic status wrapper to populate
   */
  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Validates point cloud dimensions against configured limits
   *
   * Checks if the point cloud size is within the configured min/max limits.
   * Logs warnings/errors with actual values vs limits for debugging.
   *
   * @param num_points Number of points in the point cloud
   * @return true if validation passes (within limits), false otherwise
   */
  bool validatePointCloudDimensions(size_t num_points) const;

  /**
   * @brief Filters out NaN and Inf values from a point cloud
   *
   * Creates a new point cloud matrix containing only points with finite
   * x, y, z values. This allows processing to continue even if some points
   * are invalid, rather than rejecting the entire cloud. Logs warnings if
   * invalid points are found.
   *
   * @param cloud Input point cloud (may contain NaN/Inf)
   * @return Filtered point cloud with only finite points, or empty matrix if all invalid
   */
  Eigen::MatrixX3f filterInvalidPoints(const Eigen::MatrixX3f & cloud);

  /**
   * @brief Updates processing statistics after a successful processing cycle
   *
   * Atomically updates the total processed count and cumulative processing time.
   * Updates last processing time and triggers periodic statistics logging if
   * the interval has elapsed.
   *
   * @param time_taken Processing time in milliseconds for the current cycle
   */
  void updateStatistics(double time_taken);

  /**
   * @brief Updates diagnostic information after processing a point cloud
   *
   * Ticks topic diagnostics for ground and non-ground publishers to track
   * publishing frequency and timestamp validity. Forces diagnostic updater
   * to refresh its status information.
   *
   * @param timestamp Timestamp from the processed point cloud message
   */
  void updateDiagnostics(const std_msgs::msg::Header::_stamp_type & timestamp);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;

  std::unique_ptr<GroundRemovalCore> core_;

  rclcpp::QoS subscriber_qos_;  ///< QoS profile for point cloud subscriber (configured in on_configure)
  rclcpp::QoS publisher_qos_;  ///< QoS profile for publishers (configured in on_configure)

  std::atomic<uint64_t> total_processed_{0};              ///< Total number of point clouds processed
  std::atomic<double> total_processing_time_ms_{0.0};      ///< Cumulative processing time in milliseconds
  std::atomic<double> last_processing_time_ms_{0.0};       ///< Processing time for last cloud in milliseconds
  std::chrono::steady_clock::time_point last_stats_log_time_;  ///< Last time statistics were logged
  static constexpr std::chrono::seconds kStatsLogInterval{30};  ///< Statistics logging interval

  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;  ///< Diagnostic updater for node health
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> ground_pub_diagnostic_;      ///< Topic diagnostics for ground publisher
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> nonground_pub_diagnostic_;   ///< Topic diagnostics for non-ground publisher
  double min_freq_{0.0};    ///< Minimum expected publishing frequency (Hz)
  double max_freq_{100.0};  ///< Maximum expected publishing frequency (Hz)

  PointCloudLimits point_cloud_limits_;  ///< Configurable limits for point cloud validation
};

}  // namespace wato::perception::patchworkpp
