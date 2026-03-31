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

#include "eidos/plugins/factors/liso_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include "eidos/map/map_manager.hpp"
#include "eidos/utils/conversions.hpp"
#include "eidos/utils/small_gicp_ros.hpp"

namespace eidos
{

constexpr double kInitialHessianScale = 100.0;
constexpr size_t kImuBufferMaxSize = 2500;
constexpr double kMaxGyroDt = 0.1;
constexpr double kMaxTwistDt = 1.0;
constexpr double kQuatSquaredNormMin = 0.5;
constexpr float kSubmapRebuildFraction = 0.5f;

// ==========================================================================
// Lifecycle
// ==========================================================================

void LisoFactor::onInitialize()
{
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".lidar_topic", "/lidar/points");
  node_->declare_parameter(prefix + ".odom_topic", "liso/odometry");
  node_->declare_parameter(prefix + ".odometry_incremental_topic", "liso/odometry_incremental");
  node_->declare_parameter(prefix + ".add_factors", true);
  node_->declare_parameter(prefix + ".submap_source", std::string("recent_keyframes"));
  node_->declare_parameter(prefix + ".scan_ds_resolution", scan_ds_resolution_);
  node_->declare_parameter(prefix + ".submap_ds_resolution", submap_ds_resolution_);
  node_->declare_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->declare_parameter(prefix + ".submap_radius", submap_radius_);
  node_->declare_parameter(prefix + ".max_submap_states", max_submap_states_);
  node_->declare_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->declare_parameter(prefix + ".max_iterations", max_iterations_);
  node_->declare_parameter(prefix + ".num_threads", num_threads_);
  node_->declare_parameter(prefix + ".min_inliers", min_inliers_);
  node_->declare_parameter(prefix + ".min_noise", min_noise_);
  node_->declare_parameter(prefix + ".min_scan_distance", min_scan_distance_);
  node_->declare_parameter(prefix + ".odom_pose_cov", odom_pose_cov_);
  node_->declare_parameter(prefix + ".odom_twist_cov", odom_twist_cov_);
  node_->declare_parameter(prefix + ".lidar_frame", lidar_frame_);
  node_->declare_parameter(prefix + ".imu_topic", imu_topic_);
  node_->declare_parameter(prefix + ".imu_frame", imu_frame_);
  node_->declare_parameter(prefix + ".initialization.warmup_samples", imu_warmup_samples_);
  node_->declare_parameter(prefix + ".initialization.stationary_gyr_threshold", imu_stationary_gyr_threshold_);

  std::string lidar_topic, odom_topic, odom_inc_topic;
  node_->get_parameter(prefix + ".lidar_topic", lidar_topic);
  node_->get_parameter(prefix + ".odom_topic", odom_topic);
  node_->get_parameter(prefix + ".odometry_incremental_topic", odom_inc_topic);
  node_->get_parameter(prefix + ".add_factors", add_factors_);
  node_->get_parameter(prefix + ".submap_source", submap_source_);
  node_->get_parameter(prefix + ".scan_ds_resolution", scan_ds_resolution_);
  node_->get_parameter(prefix + ".submap_ds_resolution", submap_ds_resolution_);
  node_->get_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->get_parameter(prefix + ".submap_radius", submap_radius_);
  node_->get_parameter(prefix + ".max_submap_states", max_submap_states_);
  node_->get_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->get_parameter(prefix + ".max_iterations", max_iterations_);
  node_->get_parameter(prefix + ".num_threads", num_threads_);
  node_->get_parameter(prefix + ".min_inliers", min_inliers_);
  node_->get_parameter(prefix + ".min_noise", min_noise_);
  node_->get_parameter(prefix + ".min_scan_distance", min_scan_distance_);
  node_->get_parameter(prefix + ".odom_pose_cov", odom_pose_cov_);
  node_->get_parameter(prefix + ".odom_twist_cov", odom_twist_cov_);
  node_->get_parameter(prefix + ".lidar_frame", lidar_frame_);
  node_->get_parameter(prefix + ".imu_topic", imu_topic_);
  node_->get_parameter(prefix + ".imu_frame", imu_frame_);
  node_->get_parameter(prefix + ".initialization.warmup_samples", imu_warmup_samples_);
  node_->get_parameter(prefix + ".initialization.stationary_gyr_threshold", imu_stationary_gyr_threshold_);

  node_->get_parameter("frames.map", map_frame_);
  node_->get_parameter("frames.odometry", odom_frame_);
  node_->get_parameter("frames.base_link", base_link_frame_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic, rclcpp::SensorDataQoS(), std::bind(&LisoFactor::lidarCallback, this, std::placeholders::_1), sub_opts);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(), std::bind(&LisoFactor::imuCallback, this, std::placeholders::_1), sub_opts);

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  odom_incremental_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_inc_topic, 10);
  submap_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("liso/submap", rclcpp::QoS(1).transient_local());

  // Register data formats with MapManager for persistence
  map_manager_->registerKeyframeFormat("liso_factor/cloud", "pcl_pcd_binary");
  map_manager_->registerKeyframeFormat("liso_factor/gicp_cloud", "small_gicp_binary");

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (GICP scan-to-submap)", name_.c_str());
}

void LisoFactor::activate()
{
  active_ = true;
  odom_pub_->on_activate();
  odom_incremental_pub_->on_activate();
  submap_pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void LisoFactor::deactivate()
{
  active_ = false;
  if (submap_rebuild_thread_.joinable()) submap_rebuild_thread_.join();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

// ==========================================================================
// onTrackingBegin — build initial submap when submap_source is "prior_map"
// ==========================================================================

void LisoFactor::onTrackingBegin(const gtsam::Pose3 & initial_pose)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "\033[33m[%s] onTrackingBegin called: pos=(%.2f,%.2f,%.2f) rpy=(%.2f,%.2f,%.2f) "
    "submap_source=%s add_factors=%d\033[0m",
    name_.c_str(),
    initial_pose.translation().x(),
    initial_pose.translation().y(),
    initial_pose.translation().z(),
    initial_pose.rotation().roll() * 180.0 / M_PI,
    initial_pose.rotation().pitch() * 180.0 / M_PI,
    initial_pose.rotation().yaw() * 180.0 / M_PI,
    submap_source_.c_str(),
    add_factors_);
  // Reset incremental odom state so it initializes fresh after relocalization
  first_scan_.store(true);
  has_prev_incremental_ = false;
  has_last_match_ = false;
  has_last_factor_ = false;
  has_prev_liso_ = false;
  has_pending_incremental_correction_.store(false, std::memory_order_relaxed);

  if (submap_source_ != "prior_map") return;

  Eigen::Vector3f pos(
    static_cast<float>(initial_pose.translation().x()),
    static_cast<float>(initial_pose.translation().y()),
    static_cast<float>(initial_pose.translation().z()));
  submap_center_ = pos;
  rebuildSubmapAtPosition(pos);
  prior_map_submap_initialized_ = true;
  last_matched_pose_ = initial_pose;
  has_last_match_ = true;

  // Seed the lock-free pose outputs so eidos_transform has data immediately
  setMapPose(initial_pose);
  setOdomPose(gtsam::Pose3());

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] onTrackingBegin: built initial prior map submap at (%.2f, %.2f, %.2f)",
    name_.c_str(),
    pos.x(),
    pos.y(),
    pos.z());
}

// ==========================================================================
// produceFactor — return BetweenFactor from cached GICP result
// ==========================================================================

StampedFactorResult LisoFactor::produceFactor(gtsam::Key key, double /*timestamp*/)
{
  StampedFactorResult result;
  if (!active_ || !add_factors_) return result;

  std::lock_guard lock(result_mtx_);
  if (!has_cached_result_) return result;

  // Consume the cached result
  has_cached_result_ = false;
  gtsam::Point3 current_pos = cached_pose_.translation();

  result.timestamp = cached_timestamp_;
  result.values.insert(key, cached_pose_);

  // Store body-frame clouds in MapManager
  map_manager_->store(key, "liso_factor/gicp_cloud", cached_cloud_);
  map_manager_->store(key, "liso_factor/cloud", cached_pcl_cloud_);

  // BetweenFactor: relative LiDAR odometry from previous LISO state
  if (has_prev_liso_) {
    auto noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << odom_pose_cov_[3],
                                                         odom_pose_cov_[4],
                                                         odom_pose_cov_[5],
                                                         odom_pose_cov_[0],
                                                         odom_pose_cov_[1],
                                                         odom_pose_cov_[2])
                                                          .finished());
    gtsam::Pose3 relative_pose = prev_liso_pose_.between(cached_pose_);
    result.factors.push_back(
      gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(prev_liso_key_, key, relative_pose, noise));
  }

  // Update tracking
  last_factor_position_ = current_pos;
  has_last_factor_ = true;
  prev_liso_key_ = key;
  prev_liso_pose_ = cached_pose_;
  has_prev_liso_ = true;

  gtsam::Symbol sym(key);
  RCLCPP_INFO(
    node_->get_logger(),
    "\033[36m[%s] %s at (%c,%lu) pos=(%.2f,%.2f,%.2f) inliers=%zu\033[0m",
    name_.c_str(),
    result.factors.empty() ? "origin" : "BetweenFactor",
    sym.chr(),
    sym.index(),
    cached_pose_.translation().x(),
    cached_pose_.translation().y(),
    cached_pose_.translation().z(),
    cached_result_.num_inliers);

  return result;
}

// ==========================================================================
// onOptimizationComplete — re-anchor poses after graph correction
// ==========================================================================

void LisoFactor::onOptimizationComplete(const gtsam::Values & optimized_values, bool graph_corrected)
{
  if (has_prev_liso_ && optimized_values.exists(prev_liso_key_)) {
    gtsam::Pose3 corrected = optimized_values.at<gtsam::Pose3>(prev_liso_key_);

    // Store correction delta for the lidar callback to apply to prev_incremental_pose_.
    // This avoids a data race — the correction is consumed in the same thread that
    // owns prev_incremental_pose_.
    {
      std::lock_guard lock(incremental_correction_mtx_);
      pending_incremental_correction_ = prev_liso_pose_.between(corrected);
      has_pending_incremental_correction_.store(true, std::memory_order_release);
    }

    prev_liso_pose_ = corrected;
    last_matched_pose_ = corrected;

    if (graph_corrected) {
      submap_stale_ = true;
    }
  }

  // Rebuild submap with corrected poses
  rebuildSubmap();
}

// ==========================================================================
// LiDAR callback — GICP scan matching + odometry publishing
// ==========================================================================

void LisoFactor::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!active_) return;

  if (!scan_received_) {
    scan_received_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] first LiDAR scan received (%zu points)",
      name_.c_str(),
      static_cast<size_t>(msg->width * msg->height));
  }

  if (state_->load(std::memory_order_acquire) != SlamState::TRACKING) return;

  // Resolve base_link ← lidar TF (once)
  if (!has_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, lidar_frame_, tf2::TimePointZero);
      const auto & t = tf_msg.transform.translation;
      const auto & r = tf_msg.transform.rotation;
      T_base_lidar_ = Eigen::Isometry3d::Identity();
      T_base_lidar_.translation() = Eigen::Vector3d(t.x, t.y, t.z);
      T_base_lidar_.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      has_tf_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "[%s] waiting for TF %s <- %s: %s",
        name_.c_str(),
        base_link_frame_.c_str(),
        lidar_frame_.c_str(),
        ex.what());
      return;
    }
  }

  // Convert + transform to body frame
  auto raw_cloud = fromRosMsg(*msg);
  if (raw_cloud->empty()) return;

  auto pcl_cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  pcl_cloud->reserve(raw_cloud->size());
  for (size_t i = 0; i < raw_cloud->size(); i++) {
    Eigen::Vector4d & pt = raw_cloud->point(i);
    Eigen::Vector3d p = T_base_lidar_ * pt.head<3>();
    pt.head<3>() = p;
    PointType pcl_pt;
    pcl_pt.x = static_cast<float>(p.x());
    pcl_pt.y = static_cast<float>(p.y());
    pcl_pt.z = static_cast<float>(p.z());
    pcl_pt.intensity = 0.0f;
    pcl_cloud->push_back(pcl_pt);
  }

  // Preprocess: downsample + normals + covariances
  auto [scan, scan_tree] = small_gicp::preprocess_points(*raw_cloud, scan_ds_resolution_, num_neighbors_, num_threads_);
  if (scan->empty()) return;

  double scan_time = rclcpp::Time(msg->header.stamp).seconds();

  // First scan: cache and return
  if (first_scan_) {
    std::lock_guard lock(result_mtx_);
    cached_cloud_ = scan;
    cached_pcl_cloud_ = pcl_cloud;
    cached_timestamp_ = scan_time;
    // Use relocalized pose if available (set by onTrackingBegin), otherwise origin
    if (has_last_match_) {
      cached_pose_ = last_matched_pose_;
      RCLCPP_INFO(
        node_->get_logger(),
        "\033[33m[%s] first scan: using relocalized pose (%.2f,%.2f,%.2f) rpy=(%.2f,%.2f,%.2f)\033[0m",
        name_.c_str(),
        cached_pose_.translation().x(),
        cached_pose_.translation().y(),
        cached_pose_.translation().z(),
        cached_pose_.rotation().roll() * 180.0 / M_PI,
        cached_pose_.rotation().pitch() * 180.0 / M_PI,
        cached_pose_.rotation().yaw() * 180.0 / M_PI);
    } else {
      cached_pose_ = gtsam::Pose3(initial_gravity_orientation_, gtsam::Point3(0, 0, 0));
      last_matched_pose_ = cached_pose_;
      has_last_match_ = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "\033[33m[%s] first scan: using gravity-aligned origin (%.2f,%.2f,%.2f) rpy=(%.2f,%.2f,%.2f)\033[0m",
        name_.c_str(),
        cached_pose_.translation().x(),
        cached_pose_.translation().y(),
        cached_pose_.translation().z(),
        cached_pose_.rotation().roll() * 180.0 / M_PI,
        cached_pose_.rotation().pitch() * 180.0 / M_PI,
        cached_pose_.rotation().yaw() * 180.0 / M_PI);
    }
    cached_result_ = small_gicp::RegistrationResult();
    cached_result_.converged = true;
    cached_result_.num_inliers = scan->size();
    cached_result_.H = Eigen::Matrix<double, 6, 6>::Identity() * kInitialHessianScale;
    has_cached_result_ = true;
    first_scan_ = false;
    gyro_tracking_active_ = true;
    {
      std::lock_guard lock(gyro_mtx_);
      gyro_delta_rpy_ = Eigen::Vector3d::Zero();
    }
    last_gyro_time_ = scan_time;
    return;
  }

  if (submap_stale_.load()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[%s] submap is stale, skipping scan", name_.c_str());
    return;
  }

  // Initial guess from gyro-integrated rotation
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (has_last_match_) {
    Eigen::Vector3d gyro_delta;
    {
      std::lock_guard lock(gyro_mtx_);
      gyro_delta = gyro_delta_rpy_;
    }
    gtsam::Rot3 delta_rot = gtsam::Rot3::RzRyRx(gyro_delta);
    gtsam::Rot3 init_rotation = last_matched_pose_.rotation() * delta_rot;
    gtsam::Pose3 predicted(init_rotation, last_matched_pose_.translation());
    init_guess = Eigen::Isometry3d(predicted.matrix());
  }

  // GICP scan-to-submap matching
  small_gicp::RegistrationResult result;
  {
    std::shared_lock lock(submap_mtx_);
    if (!cached_submap_ || !cached_submap_tree_ || cached_submap_->empty()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[%s] no submap available for GICP matching", name_.c_str());
      return;
    }

    small_gicp::RegistrationSetting setting;
    setting.type = small_gicp::RegistrationSetting::GICP;
    setting.max_correspondence_distance = max_correspondence_distance_;
    setting.max_iterations = max_iterations_;
    setting.num_threads = num_threads_;

    result = small_gicp::align(*cached_submap_, *scan, *cached_submap_tree_, init_guess, setting);
  }

  if (!result.converged || static_cast<int>(result.num_inliers) < min_inliers_) {
    RCLCPP_WARN(node_->get_logger(),
      "[%s] GICP failed: converged=%d inliers=%zu (min=%d)",
      name_.c_str(), result.converged, result.num_inliers, min_inliers_);
    return;
  }

  gtsam::Pose3 matched_pose(
    gtsam::Rot3(result.T_target_source.rotation()), gtsam::Point3(result.T_target_source.translation()));

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    2000,
    "\033[36m[%s] GICP match: pos=(%.2f,%.2f,%.2f) yaw=%.2f° | "
    "guess: pos=(%.2f,%.2f,%.2f) yaw=%.2f° | inliers=%zu\033[0m",
    name_.c_str(),
    matched_pose.translation().x(),
    matched_pose.translation().y(),
    matched_pose.translation().z(),
    matched_pose.rotation().yaw() * 180.0 / M_PI,
    init_guess.translation().x(),
    init_guess.translation().y(),
    init_guess.translation().z(),
    gtsam::Rot3(init_guess.rotation()).yaw() * 180.0 / M_PI,
    result.num_inliers);

  // Lock-free pose outputs — eidos_transform reads these
  setMapPose(matched_pose);

  // Cache for produceFactor (when add_factors is enabled)
  if (add_factors_) {
    std::lock_guard lock(result_mtx_);
    if (!has_cached_result_) {
      bool should_cache =
        !has_last_factor_ || (matched_pose.translation() - last_factor_position_).norm() >= min_scan_distance_;
      if (should_cache) {
        cached_result_ = result;
        cached_pose_ = matched_pose;
        cached_cloud_ = scan;
        cached_pcl_cloud_ = pcl_cloud;
        cached_timestamp_ = scan_time;
        has_cached_result_ = true;
      }
    }
  }

  last_matched_pose_ = matched_pose;
  has_last_match_ = true;
  {
    std::lock_guard lock(gyro_mtx_);
    gyro_delta_rpy_ = Eigen::Vector3d::Zero();
  }
  last_gyro_time_ = scan_time;

  // Incremental odometry (odom frame)
  // Apply any pending correction from SLAM optimization before computing delta.
  if (has_pending_incremental_correction_.load(std::memory_order_acquire)) {
    std::lock_guard lock(incremental_correction_mtx_);
    prev_incremental_pose_ = prev_incremental_pose_.compose(pending_incremental_correction_);
    has_pending_incremental_correction_.store(false, std::memory_order_release);
  }

  gtsam::Pose3 delta;
  bool has_delta = false;
  double dt = (prev_odom_time_ > 0.0) ? (scan_time - prev_odom_time_) : 0.0;

  if (has_prev_incremental_) {
    delta = prev_incremental_pose_.between(matched_pose);
    incremental_pose_ = incremental_pose_.compose(delta);
    has_delta = true;
  } else {
    // In localization (prior_map), odom starts at identity — map→odom absorbs
    // the relocalized offset. In SLAM, odom ≈ map frame (no offset to absorb).
    incremental_pose_ = prior_map_submap_initialized_ ? gtsam::Pose3() : matched_pose;
    has_prev_incremental_ = true;
  }
  prev_incremental_pose_ = matched_pose;
  prev_odom_time_ = scan_time;

  // Write odom-frame pose (lock-free)
  setOdomPose(incremental_pose_);

  // Publish map-frame odometry
  if (odom_pub_->is_activated()) {
    auto q = matched_pose.rotation().toQuaternion();
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_link_frame_;
    odom_msg.pose.pose.position.x = matched_pose.translation().x();
    odom_msg.pose.pose.position.y = matched_pose.translation().y();
    odom_msg.pose.pose.position.z = matched_pose.translation().z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    for (size_t i = 0; i < odom_pose_cov_.size() && i < 6; i++) odom_msg.pose.covariance[i * 7] = odom_pose_cov_[i];
    if (has_delta && dt > 0.0 && dt < kMaxTwistDt) {
      // delta = prev.between(current) = prev^{-1} * current
      // delta.translation() = R_prev^T * (t_current - t_prev) — already in body frame
      gtsam::Point3 t = delta.translation();
      gtsam::Vector3 rpy = delta.rotation().rpy();
      odom_msg.twist.twist.linear.x = t.x() / dt;
      odom_msg.twist.twist.linear.y = t.y() / dt;
      odom_msg.twist.twist.linear.z = t.z() / dt;
      odom_msg.twist.twist.angular.x = rpy(0) / dt;
      odom_msg.twist.twist.angular.y = rpy(1) / dt;
      odom_msg.twist.twist.angular.z = rpy(2) / dt;
      for (size_t i = 0; i < odom_twist_cov_.size() && i < 6; i++)
        odom_msg.twist.covariance[i * 7] = odom_twist_cov_[i];
    }
    odom_pub_->publish(odom_msg);
  }

  // Publish odom-frame incremental odometry
  if (odom_incremental_pub_->is_activated()) {
    auto q_inc = incremental_pose_.rotation().toQuaternion();
    nav_msgs::msg::Odometry odom_inc_msg;
    odom_inc_msg.header.stamp = msg->header.stamp;
    odom_inc_msg.header.frame_id = odom_frame_;
    odom_inc_msg.child_frame_id = base_link_frame_;
    odom_inc_msg.pose.pose.position.x = incremental_pose_.translation().x();
    odom_inc_msg.pose.pose.position.y = incremental_pose_.translation().y();
    odom_inc_msg.pose.pose.position.z = incremental_pose_.translation().z();
    odom_inc_msg.pose.pose.orientation.x = q_inc.x();
    odom_inc_msg.pose.pose.orientation.y = q_inc.y();
    odom_inc_msg.pose.pose.orientation.z = q_inc.z();
    odom_inc_msg.pose.pose.orientation.w = q_inc.w();
    for (size_t i = 0; i < odom_pose_cov_.size() && i < 6; i++) odom_inc_msg.pose.covariance[i * 7] = odom_pose_cov_[i];
    if (has_delta && dt > 0.0 && dt < kMaxTwistDt) {
      // delta = prev.between(current) — translation is already in body frame
      gtsam::Point3 t = delta.translation();
      gtsam::Vector3 rpy = delta.rotation().rpy();
      odom_inc_msg.twist.twist.linear.x = t.x() / dt;
      odom_inc_msg.twist.twist.linear.y = t.y() / dt;
      odom_inc_msg.twist.twist.linear.z = t.z() / dt;
      odom_inc_msg.twist.twist.angular.x = rpy(0) / dt;
      odom_inc_msg.twist.twist.angular.y = rpy(1) / dt;
      odom_inc_msg.twist.twist.angular.z = rpy(2) / dt;
      for (size_t i = 0; i < odom_twist_cov_.size() && i < 6; i++)
        odom_inc_msg.twist.covariance[i * 7] = odom_twist_cov_[i];
    }
    odom_incremental_pub_->publish(odom_inc_msg);
  }

  // No TF broadcasting — eidos_transform reads setMapPose/setOdomPose lock-free

  // Prior map: async distance-based submap rebuild
  if (submap_source_ == "prior_map" && prior_map_submap_initialized_) {
    Eigen::Vector3f pos(
      static_cast<float>(matched_pose.translation().x()),
      static_cast<float>(matched_pose.translation().y()),
      static_cast<float>(matched_pose.translation().z()));
    if (!submap_rebuilding_.load() && (pos - submap_center_).norm() > submap_radius_ * kSubmapRebuildFraction) {
      submap_center_ = pos;
      submap_rebuilding_ = true;
      if (submap_rebuild_thread_.joinable()) submap_rebuild_thread_.join();
      submap_rebuild_thread_ = std::thread([this, pos]() {
        rebuildSubmapAtPosition(pos);
        submap_rebuilding_ = false;
      });
    }
  }
}

// ==========================================================================
// IMU callback — warmup + gyro integration for GICP initial guess
// ==========================================================================

void LisoFactor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double current_time = rclcpp::Time(msg->header.stamp).seconds();

  // Resolve base_link ← imu TF (once)
  if (!has_imu_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, imu_frame_, tf2::TimePointZero);
      const auto & r = tf_msg.transform.rotation;
      R_base_imu_ = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      has_imu_tf_ = true;
    } catch (const tf2::TransformException &) {
      return;
    }
  }

  {
    std::lock_guard lock(imu_mtx_);
    imu_buffer_.push_back(*msg);
    while (imu_buffer_.size() > kImuBufferMaxSize) imu_buffer_.pop_front();
  }

  // Warmup: stationarity + gravity alignment
  if (!imu_warmup_complete_) {
    bool has_orientation = msg->orientation_covariance[0] >= 0.0;
    double gyr_mag = std::sqrt(
      msg->angular_velocity.x * msg->angular_velocity.x + msg->angular_velocity.y * msg->angular_velocity.y +
      msg->angular_velocity.z * msg->angular_velocity.z);

    if (gyr_mag < imu_stationary_gyr_threshold_ && has_orientation) {
      Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
      if (q.squaredNorm() > kQuatSquaredNormMin) {
        q.normalize();
        if (!warmup_quat_hemisphere_set_) {
          warmup_quat_reference_ = q;
          warmup_quat_hemisphere_set_ = true;
        } else if (q.dot(warmup_quat_reference_) < 0.0) {
          q.coeffs() = -q.coeffs();
        }
        warmup_quat_sum_ += Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
        imu_warmup_count_++;
      }
    } else {
      imu_warmup_count_ = 0;
      warmup_quat_sum_ = Eigen::Vector4d::Zero();
      warmup_quat_hemisphere_set_ = false;
    }

    if (imu_warmup_count_ >= imu_warmup_samples_) {
      Eigen::Vector4d avg = warmup_quat_sum_ / imu_warmup_count_;
      Eigen::Quaterniond q_avg(avg(0), avg(1), avg(2), avg(3));
      q_avg.normalize();
      Eigen::Matrix3d R_world_base = q_avg.toRotationMatrix() * R_base_imu_.transpose();
      gtsam::Rot3 full_rot(R_world_base);
      initial_gravity_orientation_ = gtsam::Rot3::RzRyRx(0.0, full_rot.pitch(), full_rot.roll());
      imu_warmup_complete_ = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] IMU warmup complete (roll: %.4f, pitch: %.4f)",
        name_.c_str(),
        initial_gravity_orientation_.roll(),
        initial_gravity_orientation_.pitch());
    }
    last_gyro_time_ = current_time;
    return;
  }

  // Gyro integration for GICP initial guess
  if (gyro_tracking_active_ && last_gyro_time_ > 0.0) {
    double dt = current_time - last_gyro_time_;
    if (dt > 0.0 && dt < kMaxGyroDt) {
      Eigen::Vector3d omega_imu(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
      std::lock_guard lock(gyro_mtx_);
      gyro_delta_rpy_ += R_base_imu_ * omega_imu * dt;
    }
  }
  last_gyro_time_ = current_time;
}

// ==========================================================================
// Submap management
// ==========================================================================

void LisoFactor::rebuildSubmap()
{
  if (state_->load(std::memory_order_acquire) != SlamState::TRACKING) return;

  auto key_list = map_manager_->getKeyList();
  if (key_list.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[%s] rebuildSubmap: key_list empty", name_.c_str());
    return;
  }

  gtsam::Key start_key = key_list.back();
  auto collected_keys = collectRecentStates(start_key, submap_radius_);
  if (collected_keys.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[%s] rebuildSubmap: no keys within radius %.1fm", name_.c_str(), submap_radius_);
    return;
  }

  auto merged = std::make_shared<small_gicp::PointCloud>();
  auto poses_6d = map_manager_->getKeyPoses6D();

  for (gtsam::Key k : collected_keys) {
    auto body_cloud = map_manager_->retrieve<small_gicp::PointCloud::Ptr>(k, "liso_factor/gicp_cloud");
    if (!body_cloud || (*body_cloud)->empty()) continue;

    int idx = map_manager_->getCloudIndex(k);
    if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;

    const auto & pose = poses_6d->points[idx];
    Eigen::Affine3f world_T = poseTypeToAffine3f(pose);
    Eigen::Isometry3d world_T_d;
    world_T_d.matrix() = world_T.matrix().cast<double>();

    for (size_t i = 0; i < (*body_cloud)->size(); i++) {
      Eigen::Vector3d p = world_T_d * (*body_cloud)->point(i).head<3>();
      merged->points.emplace_back(p.x(), p.y(), p.z(), 1.0);
    }
  }

  if (merged->empty()) {
    RCLCPP_WARN(node_->get_logger(), "[%s] rebuildSubmap: merged cloud empty after %zu keys", name_.c_str(), collected_keys.size());
    return;
  }

  auto [submap, submap_tree] =
    small_gicp::preprocess_points(*merged, submap_ds_resolution_, num_neighbors_, num_threads_);

  {
    std::unique_lock lock(submap_mtx_);
    cached_submap_ = submap;
    cached_submap_tree_ = submap_tree;
  }
  submap_stale_ = false;

  // Publish submap for visualization
  if (submap_pub_->is_activated()) {
    pcl::PointCloud<PointType> pcl_submap;
    pcl_submap.reserve(submap->size());
    for (size_t i = 0; i < submap->size(); i++) {
      PointType pt;
      pt.x = static_cast<float>(submap->point(i).x());
      pt.y = static_cast<float>(submap->point(i).y());
      pt.z = static_cast<float>(submap->point(i).z());
      pt.intensity = 0.0f;
      pcl_submap.push_back(pt);
    }
    sensor_msgs::msg::PointCloud2 submap_msg;
    pcl::toROSMsg(pcl_submap, submap_msg);
    submap_msg.header.stamp = node_->now();
    submap_msg.header.frame_id = map_frame_;
    submap_pub_->publish(submap_msg);
  }
}

void LisoFactor::rebuildSubmapAtPosition(const Eigen::Vector3f & position)
{
  auto poses_6d = map_manager_->getKeyPoses6D();
  auto key_list = map_manager_->getKeyList();
  if (key_list.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[%s] rebuildSubmapAtPosition: key_list is empty, no keyframes loaded", name_.c_str());
    return;
  }

  auto kdtree = map_manager_->getKdTree();
  if (!kdtree) {
    RCLCPP_WARN(node_->get_logger(), "[%s] rebuildSubmapAtPosition: KdTree is null", name_.c_str());
    return;
  }

  PointType search_point;
  search_point.x = position.x();
  search_point.y = position.y();
  search_point.z = position.z();

  std::vector<int> indices;
  std::vector<float> distances;
  kdtree->radiusSearch(search_point, submap_radius_, indices, distances);

  if (static_cast<int>(indices.size()) > max_submap_states_) {
    std::vector<std::pair<float, int>> dist_idx;
    dist_idx.reserve(indices.size());
    for (size_t i = 0; i < indices.size(); i++) dist_idx.emplace_back(distances[i], indices[i]);
    std::sort(dist_idx.begin(), dist_idx.end());
    indices.clear();
    for (int i = 0; i < max_submap_states_; i++) indices.push_back(dist_idx[i].second);
  }

  auto merged = std::make_shared<small_gicp::PointCloud>();
  for (int idx : indices) {
    if (idx < 0 || idx >= static_cast<int>(key_list.size())) continue;
    gtsam::Key k = map_manager_->getKeyFromCloudIndex(idx);

    auto body_cloud = map_manager_->retrieve<small_gicp::PointCloud::Ptr>(k, "liso_factor/gicp_cloud");
    if (!body_cloud || (*body_cloud)->empty()) continue;

    if (idx >= static_cast<int>(poses_6d->size())) continue;
    const auto & pose = poses_6d->points[idx];
    Eigen::Affine3f world_T = poseTypeToAffine3f(pose);
    Eigen::Isometry3d world_T_d;
    world_T_d.matrix() = world_T.matrix().cast<double>();

    for (size_t i = 0; i < (*body_cloud)->size(); i++) {
      Eigen::Vector3d p = world_T_d * (*body_cloud)->point(i).head<3>();
      merged->points.emplace_back(p.x(), p.y(), p.z(), 1.0);
    }
  }

  if (merged->empty()) {
    RCLCPP_WARN(node_->get_logger(),
      "[%s] rebuildSubmapAtPosition: merged cloud is empty after collecting %zu keyframes",
      name_.c_str(), indices.size());
    return;
  }

  auto [submap, submap_tree] =
    small_gicp::preprocess_points(*merged, submap_ds_resolution_, num_neighbors_, num_threads_);

  {
    std::unique_lock lock(submap_mtx_);
    cached_submap_ = submap;
    cached_submap_tree_ = submap_tree;
  }
  submap_stale_ = false;

  // Publish submap for visualization
  if (submap_pub_->is_activated()) {
    pcl::PointCloud<PointType> pcl_submap;
    pcl_submap.reserve(submap->size());
    for (size_t i = 0; i < submap->size(); i++) {
      PointType pt;
      pt.x = static_cast<float>(submap->point(i).x());
      pt.y = static_cast<float>(submap->point(i).y());
      pt.z = static_cast<float>(submap->point(i).z());
      pt.intensity = 0.0f;
      pcl_submap.push_back(pt);
    }
    sensor_msgs::msg::PointCloud2 submap_msg;
    pcl::toROSMsg(pcl_submap, submap_msg);
    submap_msg.header.stamp = node_->now();
    submap_msg.header.frame_id = map_frame_;
    submap_pub_->publish(submap_msg);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] prior map submap rebuilt at (%.1f,%.1f,%.1f): %zu points",
    name_.c_str(),
    position.x(),
    position.y(),
    position.z(),
    submap->size());
}

/// Walk backwards chronologically from start, collecting states within radius.
std::vector<gtsam::Key> LisoFactor::collectRecentStates(gtsam::Key start, double radius)
{
  auto poses_6d = map_manager_->getKeyPoses6D();
  auto key_list = map_manager_->getKeyList();

  int start_idx = map_manager_->getCloudIndex(start);
  if (start_idx < 0) return {};
  const auto & start_pose = poses_6d->points[start_idx];
  Eigen::Vector3f start_pos(start_pose.x, start_pose.y, start_pose.z);

  int list_pos = -1;
  for (int i = static_cast<int>(key_list.size()) - 1; i >= 0; i--) {
    if (key_list[i] == start) {
      list_pos = i;
      break;
    }
  }
  if (list_pos < 0) return {};

  std::vector<gtsam::Key> collected;
  for (int i = list_pos; i >= 0 && static_cast<int>(collected.size()) < max_submap_states_; i--) {
    gtsam::Key k = key_list[i];
    int idx = map_manager_->getCloudIndex(k);
    if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;
    const auto & pose = poses_6d->points[idx];
    Eigen::Vector3f pos(pose.x, pose.y, pose.z);
    if ((pos - start_pos).norm() > radius) break;
    collected.push_back(k);
  }

  return collected;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::LisoFactor, eidos::FactorPlugin)
