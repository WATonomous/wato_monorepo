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

#include "eidos/plugins/relocalization/gps_icp_relocalization.hpp"

#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/buffer.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include "eidos/map/map_manager.hpp"
#include "eidos/utils/conversions.hpp"
#include "eidos/utils/small_gicp_ros.hpp"

namespace eidos
{

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void GpsIcpRelocalization::onInitialize()
{
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".gps_topic", "gps/fix");
  node_->declare_parameter(prefix + ".lidar_topic", "velodyne_points");
  node_->declare_parameter(prefix + ".imu_topic", "/imu/data");
  node_->declare_parameter(prefix + ".lidar_frame", "velodyne");
  node_->declare_parameter(prefix + ".imu_frame", "imu_link");
  node_->declare_parameter(prefix + ".gps_candidate_radius", gps_candidate_radius_);
  node_->declare_parameter(prefix + ".fitness_threshold", fitness_threshold_);
  node_->declare_parameter(prefix + ".max_icp_iterations", max_iterations_);
  node_->declare_parameter(prefix + ".scan_ds_resolution", scan_ds_resolution_);
  node_->declare_parameter(prefix + ".submap_leaf_size", submap_leaf_size_);
  node_->declare_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->declare_parameter(prefix + ".num_threads", num_threads_);
  node_->declare_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->declare_parameter(prefix + ".pointcloud_from", std::string("liso_factor"));
  node_->declare_parameter(prefix + ".gps_from", std::string("gps_factor"));

  std::string gps_topic, lidar_topic, imu_topic;
  node_->get_parameter(prefix + ".gps_topic", gps_topic);
  node_->get_parameter(prefix + ".lidar_topic", lidar_topic);
  node_->get_parameter(prefix + ".imu_topic", imu_topic);
  node_->get_parameter(prefix + ".lidar_frame", lidar_frame_);
  node_->get_parameter(prefix + ".imu_frame", imu_frame_);
  node_->get_parameter(prefix + ".gps_candidate_radius", gps_candidate_radius_);
  node_->get_parameter(prefix + ".fitness_threshold", fitness_threshold_);
  node_->get_parameter(prefix + ".max_icp_iterations", max_iterations_);
  node_->get_parameter(prefix + ".scan_ds_resolution", scan_ds_resolution_);
  node_->get_parameter(prefix + ".submap_leaf_size", submap_leaf_size_);
  node_->get_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->get_parameter(prefix + ".num_threads", num_threads_);
  node_->get_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->get_parameter(prefix + ".pointcloud_from", pointcloud_from_);
  node_->get_parameter(prefix + ".gps_from", gps_from_);
  node_->get_parameter("frames.base_link", base_link_frame_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&GpsIcpRelocalization::gpsCallback, this, std::placeholders::_1),
    sub_opts);

  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&GpsIcpRelocalization::lidarCallback, this, std::placeholders::_1),
    sub_opts);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&GpsIcpRelocalization::imuCallback, this, std::placeholders::_1),
    sub_opts);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (GPS+GICP, lidar=%s)", name_.c_str(), lidar_topic.c_str());
}

void GpsIcpRelocalization::activate()
{
  active_ = true;
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void GpsIcpRelocalization::deactivate()
{
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

// ---------------------------------------------------------------------------
// Callbacks — buffer latest sensor data
// ---------------------------------------------------------------------------
void GpsIcpRelocalization::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(gps_lock_);
  gps_queue_.push_back(*msg);
  if (gps_queue_.size() > 10) gps_queue_.pop_front();
}

void GpsIcpRelocalization::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Resolve base_link ← lidar TF (once)
  if (!has_lidar_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, lidar_frame_, tf2::TimePointZero);
      const auto & t = tf_msg.transform.translation;
      const auto & r = tf_msg.transform.rotation;
      T_base_lidar_ = Eigen::Isometry3d::Identity();
      T_base_lidar_.translation() = Eigen::Vector3d(t.x, t.y, t.z);
      T_base_lidar_.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      has_lidar_tf_ = true;
    } catch (const tf2::TransformException &) {
      return;
    }
  }

  // Convert to small_gicp and transform to body frame
  auto raw = fromRosMsg(*msg);
  if (!raw || raw->empty()) return;

  for (size_t i = 0; i < raw->size(); i++) {
    Eigen::Vector4d & pt = raw->point(i);
    pt.head<3>() = T_base_lidar_ * pt.head<3>();
  }

  auto [ds, tree] = small_gicp::preprocess_points(*raw, scan_ds_resolution_, num_neighbors_, num_threads_);

  std::lock_guard<std::mutex> lock(scan_lock_);
  latest_scan_ = ds;
}

void GpsIcpRelocalization::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Resolve base_link ← imu TF (once, rotation only)
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

  // IMU quaternion gives R_world_imu. Transform to body frame:
  // R_world_base = R_world_imu * R_imu_base = R_world_imu * inv(R_base_imu)
  Eigen::Quaterniond q_imu(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  if (q_imu.squaredNorm() < 0.5) return;
  q_imu.normalize();

  Eigen::Matrix3d R_world_base = q_imu.toRotationMatrix() * R_base_imu_.transpose();

  // Extract roll/pitch from body-frame rotation (ignore yaw — GPS doesn't give heading)
  gtsam::Rot3 body_rot(R_world_base);

  std::lock_guard<std::mutex> lock(imu_lock_);
  latest_imu_roll_ = body_rot.roll();
  latest_imu_pitch_ = body_rot.pitch();
  has_imu_ = true;
}

// ---------------------------------------------------------------------------
// tryRelocalize — match live scan against prior map submap
// ---------------------------------------------------------------------------
std::optional<RelocalizationResult> GpsIcpRelocalization::tryRelocalize(  // NOLINT(readability/casting)
  double /*timestamp*/)
{
  if (!active_) return std::nullopt;

  const auto & map_manager = (*map_manager_);
  if (!map_manager.hasPriorMap()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[%s] no prior map loaded", name_.c_str());
    return std::nullopt;
  }

  // 1. Get latest GPS fix
  sensor_msgs::msg::NavSatFix latest_fix;
  {
    std::lock_guard<std::mutex> lock(gps_lock_);
    if (gps_queue_.empty()) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[%s] waiting for GPS", name_.c_str());
      return std::nullopt;
    }
    latest_fix = gps_queue_.back();
    gps_queue_.clear();
  }

  if (latest_fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "[%s] no GPS fix (status=%d)",
      name_.c_str(),
      latest_fix.status.status);
    return std::nullopt;
  }

  // 2. Get latest live LiDAR scan
  small_gicp::PointCloud::Ptr live_scan;
  {
    std::lock_guard<std::mutex> lock(scan_lock_);
    if (!latest_scan_ || latest_scan_->empty()) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000, "[%s] waiting for LiDAR scan", name_.c_str());
      return std::nullopt;
    }
    live_scan = latest_scan_;
  }

  // 3. Convert GPS to map frame using saved utm_to_map offset
  auto offset_opt = map_manager.retrieveGlobal<std::array<double, 5>>(gps_from_ + "/utm_to_map");
  if (!offset_opt.has_value()) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "[%s] no utm_to_map offset in prior map", name_.c_str());
    return std::nullopt;
  }

  auto offset_arr = *offset_opt;
  Eigen::Vector3d utm_bias(offset_arr[0], offset_arr[1], offset_arr[2]);
  double saved_yaw = offset_arr[4];
  double cy = std::cos(-saved_yaw);
  double sy = std::sin(-saved_yaw);
  Eigen::Matrix3d R_map_enu;
  R_map_enu << cy, -sy, 0, sy, cy, 0, 0, 0, 1;

  UtmCoordinate utm = latLonToUtm(latest_fix.latitude, latest_fix.longitude, latest_fix.altitude);
  Eigen::Vector3d utm_pos(utm.easting, utm.northing, utm.altitude);
  Eigen::Vector3d map_guess = R_map_enu * utm_pos - utm_bias;

  // 4. Find prior map keyframes near GPS position
  auto key_poses_3d = map_manager.getKeyPoses3D();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  if (key_poses_3d->empty()) return std::nullopt;

  auto kdtree = (*map_manager_).getKdTree();
  if (!kdtree) return std::nullopt;

  PointType search_point;
  search_point.x = static_cast<float>(map_guess.x());
  search_point.y = static_cast<float>(map_guess.y());
  search_point.z = static_cast<float>(map_guess.z());

  std::vector<int> candidate_indices;
  std::vector<float> candidate_distances;
  kdtree->radiusSearch(search_point, gps_candidate_radius_, candidate_indices, candidate_distances, 0);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] GPS map_guess=(%.1f,%.1f,%.1f) | %zu candidates within %.0fm | scan=%zu pts",
    name_.c_str(),
    map_guess.x(),
    map_guess.y(),
    map_guess.z(),
    candidate_indices.size(),
    gps_candidate_radius_,
    live_scan->size());

  if (candidate_indices.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[%s] no prior keyframes near GPS position", name_.c_str());
    return std::nullopt;
  }

  // 5. Assemble world-frame submap from candidate clouds
  auto submap_merged = std::make_shared<small_gicp::PointCloud>();
  int best_candidate = candidate_indices[0];

  for (int idx : candidate_indices) {
    gtsam::Key idx_key = map_manager.getKeyFromCloudIndex(idx);
    if (idx_key == 0) continue;

    auto cloud = map_manager.retrieve<pcl::PointCloud<PointType>::Ptr>(idx_key, pointcloud_from_);
    if (!cloud || (*cloud)->empty()) continue;

    Eigen::Affine3f world_T = poseTypeToAffine3f(key_poses_6d->points[idx]);
    Eigen::Isometry3d T;
    T.matrix() = world_T.matrix().cast<double>();
    for (size_t i = 0; i < (*cloud)->size(); i++) {
      Eigen::Vector3d p = T * Eigen::Vector3d((*cloud)->points[i].x, (*cloud)->points[i].y, (*cloud)->points[i].z);
      submap_merged->points.emplace_back(p.x(), p.y(), p.z(), 1.0);
    }
  }

  if (submap_merged->empty()) {
    RCLCPP_INFO(node_->get_logger(), "[%s] assembled submap is empty (no cloud data)", name_.c_str());
    return std::nullopt;
  }

  // Preprocess submap
  auto [submap, submap_tree] =
    small_gicp::preprocess_points(*submap_merged, submap_leaf_size_, num_neighbors_, num_threads_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] submap: %zu raw -> %zu preprocessed pts",
    name_.c_str(),
    submap_merged->size(),
    submap->size());

  // 6. Build init_guess: GPS position + IMU roll/pitch + nearest keyframe yaw
  double init_roll = 0.0, init_pitch = 0.0;
  {
    std::lock_guard<std::mutex> lock(imu_lock_);
    if (has_imu_) {
      init_roll = latest_imu_roll_;
      init_pitch = latest_imu_pitch_;
    }
  }

  // Use nearest prior keyframe's yaw as initial heading guess
  double init_yaw = key_poses_6d->points[best_candidate].yaw;

  gtsam::Pose3 init_pose(
    gtsam::Rot3::RzRyRx(init_roll, init_pitch, init_yaw), gtsam::Point3(map_guess.x(), map_guess.y(), map_guess.z()));
  Eigen::Isometry3d init_guess;
  init_guess.matrix() = init_pose.matrix();

  // 7. GICP: align live scan (body frame) -> submap (world frame)
  // Result T_target_source IS the robot's world-frame pose
  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.max_correspondence_distance = max_correspondence_distance_;
  setting.max_iterations = max_iterations_;
  setting.num_threads = num_threads_;

  auto result = small_gicp::align(*submap, *live_scan, *submap_tree, init_guess, setting);

  if (!result.converged) {
    RCLCPP_INFO(node_->get_logger(), "[%s] GICP did not converge", name_.c_str());
    return std::nullopt;
  }

  double inlier_ratio = static_cast<double>(result.num_inliers) / live_scan->size();

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] GICP: converged, inliers=%zu/%zu (%.1f%%), error=%.4f (threshold=%.3f)",
    name_.c_str(),
    result.num_inliers,
    live_scan->size(),
    inlier_ratio * 100.0,
    result.error,
    fitness_threshold_);

  if (inlier_ratio < 0.3) {
    RCLCPP_INFO(node_->get_logger(), "[%s] inlier ratio too low", name_.c_str());
    return std::nullopt;
  }

  // 8. Extract relocalized pose
  gtsam::Pose3 relocalized_pose(
    gtsam::Rot3(result.T_target_source.rotation()), gtsam::Point3(result.T_target_source.translation()));

  auto t = relocalized_pose.translation();
  auto rpy = relocalized_pose.rotation().rpy();
  RCLCPP_INFO(
    node_->get_logger(),
    "\033[32m[%s] RELOCALIZED: pos=(%.1f,%.1f,%.1f) rpy=(%.1f,%.1f,%.1f) "
    "inliers=%zu (%.0f%%)\033[0m",
    name_.c_str(),
    t.x(),
    t.y(),
    t.z(),
    rpy(0) * 180.0 / M_PI,
    rpy(1) * 180.0 / M_PI,
    rpy(2) * 180.0 / M_PI,
    result.num_inliers,
    inlier_ratio * 100.0);

  // 9. Refine utm_to_map offset using the precise relocalized position
  Eigen::Vector3d refined_bias = R_map_enu * utm_pos - Eigen::Vector3d(t.x(), t.y(), t.z());
  double zone_encoded = static_cast<double>(utm.zone * 10 + (utm.is_north ? 1 : 0));
  std::array<double, 5> refined_global = {
    refined_bias.x(), refined_bias.y(), refined_bias.z(), zone_encoded, saved_yaw};
  map_manager_->storeGlobal("gps_factor/utm_to_map", refined_global);

  RelocalizationResult reloc_result;
  reloc_result.pose = relocalized_pose;
  reloc_result.fitness_score = result.error;
  reloc_result.matched_keyframe_index = best_candidate;
  return reloc_result;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsIcpRelocalization, eidos::RelocalizationPlugin)
