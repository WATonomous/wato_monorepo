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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/point_cloud.h>

#include <atomic>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_result.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/utils/types.hpp"

namespace eidos
{

/**
 * @brief LiDAR-Inertial Submap Odometry factor plugin.
 *
 * Subscribes to LiDAR + IMU. Performs GICP scan-to-submap matching to produce:
 * - BetweenFactor<Pose3> for the SLAM graph (via produceFactor)
 * - Map-frame and odom-frame poses (via setMapPose / setOdomPose, lock-free)
 * - Odometry topics (map-frame absolute + odom-frame incremental)
 *
 * Does NOT broadcast TF — eidos_transform reads poses lock-free.
 *
 * Threading:
 * - LiDAR callback: GICP matching, writes lock-free poses, caches factors.
 * - IMU callback: warmup detection + gyro integration for initial guess.
 * - SLAM loop: reads produceFactor() (lock-free via AtomicSlot).
 * - Submap rebuild: synchronous in lidarCallback (fixed window, fast).
 */
class LisoFactor : public FactorPlugin
{
public:
  LisoFactor() = default;
  ~LisoFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  StampedFactorResult produceFactor(gtsam::Key key, double timestamp) override;
  void onTrackingBegin(const gtsam::Pose3 & initial_pose) override;
  void onOptimizationComplete(const gtsam::Values & optimized_values, bool loop_closure_detected) override;

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void rebuildSubmap();
  void rebuildSubmapAtPosition(const Eigen::Vector3f & position);
  std::vector<gtsam::Key> collectRecentStates(gtsam::Key start, double radius);

  // ---- State flags ----
  std::atomic<bool> first_scan_{true};
  std::atomic<bool> active_{false};
  std::atomic<bool> scan_received_{false};
  std::atomic<bool> submap_stale_{false};

  // ---- Cached submap ----
  small_gicp::PointCloud::Ptr cached_submap_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> cached_submap_tree_;
  mutable std::shared_mutex submap_mtx_;

  // ---- Cached GICP result for produceFactor ----
  small_gicp::RegistrationResult cached_result_;
  gtsam::Pose3 cached_pose_;
  small_gicp::PointCloud::Ptr cached_cloud_;
  pcl::PointCloud<PointType>::Ptr cached_pcl_cloud_;
  double cached_timestamp_ = 0.0;
  bool has_cached_result_ = false;
  mutable std::mutex result_mtx_;

  // ---- Subscriptions + publishers ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_incremental_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_pub_;

  // ---- Last match tracking (for GICP initial guess) ----
  gtsam::Pose3 last_matched_pose_;
  bool has_last_match_ = false;

  // ---- IMU buffer + warmup ----
  std::deque<sensor_msgs::msg::Imu> imu_buffer_;
  std::mutex imu_mtx_;
  Eigen::Vector3d gyro_delta_rpy_{0, 0, 0};
  double last_gyro_time_ = 0.0;
  bool gyro_tracking_active_ = false;
  Eigen::Matrix3d R_base_imu_ = Eigen::Matrix3d::Identity();
  bool has_imu_tf_ = false;
  std::atomic<bool> imu_warmup_complete_{false};
  int imu_warmup_count_ = 0;
  int imu_warmup_samples_ = 200;
  double imu_stationary_gyr_threshold_ = 0.005;
  Eigen::Vector4d warmup_quat_sum_{0, 0, 0, 0};
  bool warmup_quat_hemisphere_set_ = false;
  Eigen::Quaterniond warmup_quat_reference_;
  gtsam::Rot3 initial_gravity_orientation_;

  // ---- Incremental odometry (odom frame, never corrected) ----
  gtsam::Pose3 incremental_pose_;
  gtsam::Pose3 prev_incremental_pose_;
  bool has_prev_incremental_ = false;
  double prev_odom_time_ = 0.0;

  // ---- Factor tracking ----
  gtsam::Point3 last_factor_position_ = gtsam::Point3(0, 0, 0);
  bool has_last_factor_ = false;
  gtsam::Key prev_liso_key_ = 0;
  gtsam::Pose3 prev_liso_pose_;
  bool has_prev_liso_ = false;

  // ---- Localization submap ----
  Eigen::Vector3f submap_center_{0, 0, 0};
  bool prior_map_submap_initialized_ = false;
  std::thread submap_rebuild_thread_;
  std::atomic<bool> submap_rebuilding_{false};

  // ---- Extrinsics ----
  Eigen::Isometry3d T_base_lidar_ = Eigen::Isometry3d::Identity();
  bool has_tf_ = false;

  // ---- Config ----
  bool add_factors_ = true;  ///< Whether to cache GICP results for factor graph
  std::string submap_source_ = "recent_keyframes";  ///< "recent_keyframes" or "prior_map"
  double scan_ds_resolution_ = 0.5;
  double submap_ds_resolution_ = 0.5;
  int num_neighbors_ = 10;
  double submap_radius_ = 20.0;
  int max_submap_states_ = 10;
  double max_correspondence_distance_ = 2.0;
  int max_iterations_ = 20;
  int num_threads_ = 16;
  int min_inliers_ = 50;
  double min_noise_ = 0.01;
  double min_scan_distance_ = 5.0;
  std::vector<double> odom_pose_cov_ = {0.01, 0.01, 0.005, 1e-6, 1e-6, 1e-3};
  std::vector<double> odom_twist_cov_ = {1e-4, 1e-4, 1e-8, 1e-10, 1e-10, 1e-6};
  std::string lidar_frame_ = "lidar";
  std::string imu_frame_ = "imu_link";
  std::string imu_topic_ = "/imu/data";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string base_link_frame_ = "base_link";
};

}  // namespace eidos
