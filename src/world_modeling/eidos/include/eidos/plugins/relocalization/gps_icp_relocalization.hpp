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

#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

#include "eidos/plugins/base_relocalization_plugin.hpp"
#include "eidos/utils/types.hpp"
#include "eidos/utils/utm.hpp"

namespace eidos
{

/**
 * @brief GPS + ICP relocalization against a prior map.
 *
 * Subscribes to live GPS, LiDAR, and IMU. Uses GPS to find candidate
 * keyframes in the prior map, assembles a world-frame submap from their
 * clouds, then aligns the live LiDAR scan against that submap via GICP.
 * The init_guess comes from GPS position + IMU gravity + nearest keyframe yaw.
 */
class GpsIcpRelocalization : public RelocalizationPlugin
{
public:
  GpsIcpRelocalization() = default;
  ~GpsIcpRelocalization() override = default;

  /// @brief Declare ROS parameters, create GPS/LiDAR/IMU subscriptions.
  void onInitialize() override;

  /// @brief Enable sensor subscriptions for relocalization.
  void activate() override;

  /// @brief Disable sensor subscriptions.
  void deactivate() override;

  /**
   * @brief Attempt GPS + ICP relocalization against the prior map.
   *
   * Uses the latest GPS fix to find candidate keyframes in the prior map,
   * assembles a world-frame submap from their clouds, then aligns the latest
   * live LiDAR scan against it via GICP. The initial guess combines GPS
   * position, IMU gravity alignment, and nearest keyframe yaw.
   *
   * @param timestamp Current SLAM cycle timestamp (seconds).
   * @return RelocalizationResult if GICP converges with acceptable fitness, std::nullopt otherwise.
   */
  std::optional<RelocalizationResult> tryRelocalize(double timestamp) override;

private:
  /**
   * @brief Buffer incoming GPS fixes for candidate keyframe lookup.
   * @param msg Incoming NavSatFix message.
   */
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /**
   * @brief Downsample and buffer the latest LiDAR scan for GICP alignment.
   * @param msg Incoming PointCloud2 message.
   */
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Extract roll/pitch from IMU orientation for gravity-aligned initial guess.
   * @param msg Incoming IMU message.
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Buffered sensor data
  std::deque<sensor_msgs::msg::NavSatFix> gps_queue_;
  std::mutex gps_lock_;

  small_gicp::PointCloud::Ptr latest_scan_;
  std::mutex scan_lock_;

  double latest_imu_roll_ = 0.0;
  double latest_imu_pitch_ = 0.0;
  std::mutex imu_lock_;
  bool has_imu_ = false;

  bool active_ = false;

  // base_link ← lidar TF (resolved once)
  Eigen::Isometry3d T_base_lidar_ = Eigen::Isometry3d::Identity();
  bool has_lidar_tf_ = false;

  // base_link ← imu TF (resolved once)
  Eigen::Matrix3d R_base_imu_ = Eigen::Matrix3d::Identity();
  bool has_imu_tf_ = false;

  std::string base_link_frame_;
  std::string imu_frame_;

  // Parameters
  double gps_candidate_radius_ = 30.0;
  double fitness_threshold_ = 0.3;
  int max_iterations_ = 100;
  double scan_ds_resolution_ = 0.5;
  double submap_leaf_size_ = 0.4;
  double max_correspondence_distance_ = 2.0;
  int num_threads_ = 4;
  int num_neighbors_ = 10;
  std::string pointcloud_from_;
  std::string gps_from_;
  std::string lidar_frame_;
};

}  // namespace eidos
