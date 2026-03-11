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

#include "lidar_aggregator/lidar_aggregator_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_aggregator
{

namespace
{

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    // Large primes for low collision rate on integer grid coordinates.
    const std::size_t h1 = static_cast<std::size_t>(key.x) * 73856093u;
    const std::size_t h2 = static_cast<std::size_t>(key.y) * 19349663u;
    const std::size_t h3 = static_cast<std::size_t>(key.z) * 83492791u;
    return h1 ^ h2 ^ h3;
  }
};

VoxelKey make_voxel_key(const Eigen::Vector3d & point, double voxel_size)
{
  return VoxelKey{
    static_cast<int>(std::floor(point.x() / voxel_size)),
    static_cast<int>(std::floor(point.y() / voxel_size)),
    static_cast<int>(std::floor(point.z() / voxel_size))};
}

double abs_time_delta_sec(const rclcpp::Time & a, const rclcpp::Time & b)
{
  return std::abs((a - b).seconds());
}

}  // namespace

LidarAggregatorNode::LidarAggregatorNode(const rclcpp::NodeOptions & options)
: Node("lidar_aggregator", options)
{
  declare_and_load_parameters();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth_)));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_,
    qos,
    std::bind(&LidarAggregatorNode::imu_callback, this, std::placeholders::_1));

  sub_cc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    center_topic_,
    qos,
    std::bind(&LidarAggregatorNode::lidar_cc_callback, this, std::placeholders::_1));

  sub_ne_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    ne_topic_,
    qos,
    std::bind(&LidarAggregatorNode::lidar_ne_callback, this, std::placeholders::_1));

  sub_nw_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    nw_topic_,
    qos,
    std::bind(&LidarAggregatorNode::lidar_nw_callback, this, std::placeholders::_1));

  pub_ne_deskewed_cc_ = create_publisher<sensor_msgs::msg::PointCloud2>(deskewed_ne_output_topic_, qos);
  pub_nw_deskewed_cc_ = create_publisher<sensor_msgs::msg::PointCloud2>(deskewed_nw_output_topic_, qos);
  pub_merged_ = create_publisher<sensor_msgs::msg::PointCloud2>(merged_output_topic_, qos);
  pub_estimated_offsets_ =
    create_publisher<geometry_msgs::msg::Vector3Stamped>(estimated_offsets_output_topic_, qos);
  pub_offset_scores_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(offset_scores_output_topic_, qos);

  RCLCPP_INFO(
    get_logger(),
    "LidarAggregatorNode running. Inputs: cc=%s ne=%s nw=%s imu=%s",
    center_topic_.c_str(),
    ne_topic_.c_str(),
    nw_topic_.c_str(),
    imu_topic_.c_str());
}

void LidarAggregatorNode::declare_and_load_parameters()
{
  declare_parameter<std::string>("topics.imu", "/novatel/oem7/imu/data");
  declare_parameter<std::string>("topics.center", "/lidar_cc/velodyne_points");
  declare_parameter<std::string>("topics.ne", "/lidar_ne/velodyne_points");
  declare_parameter<std::string>("topics.nw", "/lidar_nw/velodyne_points");

  declare_parameter<std::string>("outputs.ne_deskewed_cc", "/lidar_ne/points_deskewed_cc");
  declare_parameter<std::string>("outputs.nw_deskewed_cc", "/lidar_nw/points_deskewed_cc");
  declare_parameter<std::string>("outputs.merged", "/lidar/all/points_merged");
  declare_parameter<std::string>("outputs.estimated_offsets", "/lidar/sync/estimated_offsets");
  declare_parameter<std::string>("outputs.offset_scores", "/lidar/sync/offset_scores");

  declare_parameter<std::string>("frames.center", "lidar_cc");
  declare_parameter<int>("runtime.qos_depth", 20);
  declare_parameter<double>("runtime.max_pair_dt_sec", 0.08);
  declare_parameter<double>("runtime.max_imu_buffer_sec", 20.0);
  declare_parameter<double>("runtime.max_imu_interp_gap_sec", 0.05);

  declare_parameter<double>("timing.ne_time_offset_sec", 0.0);
  declare_parameter<double>("timing.nw_time_offset_sec", 0.0);

  declare_parameter<bool>("estimation.enable_online_offset", false);
  declare_parameter<bool>("estimation.write_back_to_params", true);
  declare_parameter<double>("estimation.search_half_window_sec", 0.03);
  declare_parameter<double>("estimation.search_step_sec", 0.003);
  declare_parameter<double>("estimation.ema_alpha", 0.2);
  declare_parameter<double>("estimation.min_yaw_rate_rad_s", 0.05);
  declare_parameter<double>("estimation.min_score_gain", 0.01);
  declare_parameter<double>("estimation.voxel_size_m", 0.5);
  declare_parameter<int>("estimation.min_points", 300);
  declare_parameter<double>("estimation.min_offset_sec", -0.1);
  declare_parameter<double>("estimation.max_offset_sec", 0.1);

  declare_parameter<double>("extrinsic.ne.xyz.x", 0.924884);
  declare_parameter<double>("extrinsic.ne.xyz.y", -0.972881);
  declare_parameter<double>("extrinsic.ne.xyz.z", -0.840734);
  declare_parameter<double>("extrinsic.ne.rpy.roll", 0.017163);
  declare_parameter<double>("extrinsic.ne.rpy.pitch", 0.108708);
  declare_parameter<double>("extrinsic.ne.rpy.yaw", -0.580009);

  declare_parameter<double>("extrinsic.nw.xyz.x", 1.102008);
  declare_parameter<double>("extrinsic.nw.xyz.y", 0.748385);
  declare_parameter<double>("extrinsic.nw.xyz.z", -0.824361);
  declare_parameter<double>("extrinsic.nw.rpy.roll", 0.049416);
  declare_parameter<double>("extrinsic.nw.rpy.pitch", 0.068287);
  declare_parameter<double>("extrinsic.nw.rpy.yaw", 0.246267);

  get_parameter("topics.imu", imu_topic_);
  get_parameter("topics.center", center_topic_);
  get_parameter("topics.ne", ne_topic_);
  get_parameter("topics.nw", nw_topic_);

  get_parameter("outputs.ne_deskewed_cc", deskewed_ne_output_topic_);
  get_parameter("outputs.nw_deskewed_cc", deskewed_nw_output_topic_);
  get_parameter("outputs.merged", merged_output_topic_);
  get_parameter("outputs.estimated_offsets", estimated_offsets_output_topic_);
  get_parameter("outputs.offset_scores", offset_scores_output_topic_);

  get_parameter("frames.center", center_frame_);
  get_parameter("runtime.qos_depth", qos_depth_);
  get_parameter("runtime.max_pair_dt_sec", max_pair_dt_sec_);
  get_parameter("runtime.max_imu_buffer_sec", max_imu_buffer_sec_);
  get_parameter("runtime.max_imu_interp_gap_sec", max_imu_interp_gap_sec_);
  get_parameter("timing.ne_time_offset_sec", ne_time_offset_sec_);
  get_parameter("timing.nw_time_offset_sec", nw_time_offset_sec_);
  get_parameter("estimation.enable_online_offset", estimator_cfg_.enabled);
  get_parameter("estimation.write_back_to_params", estimator_cfg_.write_back_to_params);
  get_parameter("estimation.search_half_window_sec", estimator_cfg_.search_half_window_sec);
  get_parameter("estimation.search_step_sec", estimator_cfg_.search_step_sec);
  get_parameter("estimation.ema_alpha", estimator_cfg_.ema_alpha);
  get_parameter("estimation.min_yaw_rate_rad_s", estimator_cfg_.min_yaw_rate_rad_s);
  get_parameter("estimation.min_score_gain", estimator_cfg_.min_score_gain);
  get_parameter("estimation.voxel_size_m", estimator_cfg_.voxel_size_m);
  get_parameter("estimation.min_points", estimator_cfg_.min_points);
  get_parameter("estimation.min_offset_sec", estimator_cfg_.min_offset_sec);
  get_parameter("estimation.max_offset_sec", estimator_cfg_.max_offset_sec);

  double ne_x = 0.0, ne_y = 0.0, ne_z = 0.0, ne_r = 0.0, ne_p = 0.0, ne_yaw = 0.0;
  double nw_x = 0.0, nw_y = 0.0, nw_z = 0.0, nw_r = 0.0, nw_p = 0.0, nw_yaw = 0.0;

  get_parameter("extrinsic.ne.xyz.x", ne_x);
  get_parameter("extrinsic.ne.xyz.y", ne_y);
  get_parameter("extrinsic.ne.xyz.z", ne_z);
  get_parameter("extrinsic.ne.rpy.roll", ne_r);
  get_parameter("extrinsic.ne.rpy.pitch", ne_p);
  get_parameter("extrinsic.ne.rpy.yaw", ne_yaw);

  get_parameter("extrinsic.nw.xyz.x", nw_x);
  get_parameter("extrinsic.nw.xyz.y", nw_y);
  get_parameter("extrinsic.nw.xyz.z", nw_z);
  get_parameter("extrinsic.nw.rpy.roll", nw_r);
  get_parameter("extrinsic.nw.rpy.pitch", nw_p);
  get_parameter("extrinsic.nw.rpy.yaw", nw_yaw);

  t_center_ne_ = rigid_from_xyz_rpy(ne_x, ne_y, ne_z, ne_r, ne_p, ne_yaw);
  t_center_nw_ = rigid_from_xyz_rpy(nw_x, nw_y, nw_z, nw_r, nw_p, nw_yaw);

  estimator_cfg_.search_step_sec = std::max(estimator_cfg_.search_step_sec, 1e-4);
  estimator_cfg_.ema_alpha = std::clamp(estimator_cfg_.ema_alpha, 0.0, 1.0);
}

void LidarAggregatorNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  Eigen::Quaterniond q(
    msg->orientation.w,
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z);
  if (q.norm() < 1e-6) {
    return;
  }
  q.normalize();

  std::lock_guard<std::mutex> lock(mutex_);

  imu_buffer_.push_back(ImuSample{rclcpp::Time(msg->header.stamp), q, msg->angular_velocity.z});

  if (imu_buffer_.empty()) {
    return;
  }

  const rclcpp::Time newest = imu_buffer_.back().stamp;
  while (!imu_buffer_.empty()) {
    if ((newest - imu_buffer_.front().stamp).seconds() <= max_imu_buffer_sec_) {
      break;
    }
    imu_buffer_.pop_front();
  }
}

void LidarAggregatorNode::lidar_ne_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ne_ = msg;
}

void LidarAggregatorNode::lidar_nw_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_nw_ = msg;
}

void LidarAggregatorNode::lidar_cc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  try_publish_fusion_locked(msg);
}

bool LidarAggregatorNode::get_orientation_at_time(const rclcpp::Time & stamp, Eigen::Quaterniond & q_out) const
{
  if (imu_buffer_.size() < 2) {
    return false;
  }

  if (stamp < imu_buffer_.front().stamp || stamp > imu_buffer_.back().stamp) {
    return false;
  }

  auto upper = std::lower_bound(
    imu_buffer_.begin(),
    imu_buffer_.end(),
    stamp,
    [](const ImuSample & sample, const rclcpp::Time & t) { return sample.stamp < t; });

  if (upper == imu_buffer_.begin()) {
    q_out = upper->orientation;
    return true;
  }
  if (upper == imu_buffer_.end()) {
    q_out = imu_buffer_.back().orientation;
    return true;
  }

  const auto & b = *upper;
  const auto & a = *(upper - 1);

  const double dt = (b.stamp - a.stamp).seconds();
  if (dt <= 1e-6 || dt > max_imu_interp_gap_sec_) {
    return false;
  }

  const double alpha = (stamp - a.stamp).seconds() / dt;
  q_out = a.orientation.slerp(alpha, b.orientation);
  q_out.normalize();
  return true;
}

bool LidarAggregatorNode::get_rotation_delta(
  const rclcpp::Time & from_stamp,
  const rclcpp::Time & to_stamp,
  Eigen::Matrix3d & delta_rotation) const
{
  Eigen::Quaterniond q_from;
  Eigen::Quaterniond q_to;
  if (!get_orientation_at_time(from_stamp, q_from) || !get_orientation_at_time(to_stamp, q_to)) {
    return false;
  }

  delta_rotation = (q_to.conjugate() * q_from).toRotationMatrix();
  return true;
}

bool LidarAggregatorNode::get_abs_gyro_z_at_time(const rclcpp::Time & stamp, double & abs_gyro_z_out) const
{
  if (imu_buffer_.empty()) {
    return false;
  }

  auto upper = std::lower_bound(
    imu_buffer_.begin(),
    imu_buffer_.end(),
    stamp,
    [](const ImuSample & sample, const rclcpp::Time & t) { return sample.stamp < t; });

  if (upper == imu_buffer_.begin()) {
    abs_gyro_z_out = std::abs(upper->gyro_z);
    return true;
  }
  if (upper == imu_buffer_.end()) {
    abs_gyro_z_out = std::abs(imu_buffer_.back().gyro_z);
    return true;
  }

  const auto & b = *upper;
  const auto & a = *(upper - 1);
  if (abs_time_delta_sec(stamp, a.stamp) <= abs_time_delta_sec(stamp, b.stamp)) {
    abs_gyro_z_out = std::abs(a.gyro_z);
  } else {
    abs_gyro_z_out = std::abs(b.gyro_z);
  }
  return true;
}

sensor_msgs::msg::PointCloud2::SharedPtr LidarAggregatorNode::compensate_side_cloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & side_msg,
  const rclcpp::Time & center_stamp,
  const RigidTransform & t_center_side,
  double side_time_offset_sec,
  const std::string & output_frame) const
{
  if (!side_msg) {
    return nullptr;
  }

  pcl::PointCloud<pcl::PointXYZI> side_cloud;
  pcl::fromROSMsg(*side_msg, side_cloud);

  pcl::PointCloud<pcl::PointXYZI> corrected;
  corrected.reserve(side_cloud.size());

  const rclcpp::Time side_stamp =
    rclcpp::Time(side_msg->header.stamp) + rclcpp::Duration::from_seconds(side_time_offset_sec);
  Eigen::Matrix3d r_delta = Eigen::Matrix3d::Identity();
  const bool has_delta = get_rotation_delta(side_stamp, center_stamp, r_delta);

  if (!has_delta) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "IMU interpolation unavailable for side cloud compensation. Publishing extrinsic-only transformed cloud.");
  }

  for (const auto & p : side_cloud.points) {
    Eigen::Vector3d p_side(p.x, p.y, p.z);
    Eigen::Vector3d p_center = t_center_side.rotation * p_side + t_center_side.translation;
    if (has_delta) {
      p_center = r_delta * p_center;
    }

    pcl::PointXYZI out;
    out.x = static_cast<float>(p_center.x());
    out.y = static_cast<float>(p_center.y());
    out.z = static_cast<float>(p_center.z());
    out.intensity = p.intensity;
    corrected.push_back(out);
  }

  corrected.width = static_cast<uint32_t>(corrected.size());
  corrected.height = 1;
  corrected.is_dense = false;

  auto out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(corrected, *out_msg);
  out_msg->header.stamp = center_stamp;
  out_msg->header.frame_id = output_frame;
  return out_msg;
}

sensor_msgs::msg::PointCloud2::SharedPtr LidarAggregatorNode::merge_clouds(
  const sensor_msgs::msg::PointCloud2::SharedPtr & cc,
  const sensor_msgs::msg::PointCloud2::SharedPtr & ne,
  const sensor_msgs::msg::PointCloud2::SharedPtr & nw,
  const rclcpp::Time & stamp,
  const std::string & frame_id) const
{
  pcl::PointCloud<pcl::PointXYZI> merged;

  auto append = [&merged](const sensor_msgs::msg::PointCloud2::SharedPtr & msg) {
    if (!msg) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    merged += cloud;
  };

  append(cc);
  append(ne);
  append(nw);

  merged.width = static_cast<uint32_t>(merged.size());
  merged.height = 1;
  merged.is_dense = false;

  auto out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(merged, *out_msg);
  out_msg->header.stamp = stamp;
  out_msg->header.frame_id = frame_id;
  return out_msg;
}

double LidarAggregatorNode::score_side_offset_candidate(
  const pcl::PointCloud<pcl::PointXYZI> & center_cloud,
  const pcl::PointCloud<pcl::PointXYZI> & side_cloud,
  const rclcpp::Time & center_stamp,
  const rclcpp::Time & side_stamp,
  const RigidTransform & t_center_side,
  double candidate_offset_sec) const
{
  if (center_cloud.empty() || side_cloud.empty()) {
    return 0.0;
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> center_voxels;
  center_voxels.reserve(center_cloud.size());
  for (const auto & p : center_cloud.points) {
    center_voxels.insert(make_voxel_key(Eigen::Vector3d(p.x, p.y, p.z), estimator_cfg_.voxel_size_m));
  }

  const rclcpp::Time side_aligned = side_stamp + rclcpp::Duration::from_seconds(candidate_offset_sec);
  Eigen::Matrix3d r_delta = Eigen::Matrix3d::Identity();
  const bool has_delta = get_rotation_delta(side_aligned, center_stamp, r_delta);
  if (!has_delta) {
    return 0.0;
  }

  size_t hits = 0;
  size_t total = 0;
  for (const auto & p : side_cloud.points) {
    Eigen::Vector3d p_side(p.x, p.y, p.z);
    Eigen::Vector3d p_center = t_center_side.rotation * p_side + t_center_side.translation;
    p_center = r_delta * p_center;

    if (center_voxels.find(make_voxel_key(p_center, estimator_cfg_.voxel_size_m)) != center_voxels.end()) {
      ++hits;
    }
    ++total;
  }

  if (total == 0) {
    return 0.0;
  }
  return static_cast<double>(hits) / static_cast<double>(total);
}

bool LidarAggregatorNode::maybe_update_side_offset_locked(
  const sensor_msgs::msg::PointCloud2::SharedPtr & center_msg,
  const sensor_msgs::msg::PointCloud2::SharedPtr & side_msg,
  const RigidTransform & t_center_side,
  double & side_time_offset_sec,
  double & last_score_out,
  const char * side_name,
  const char * side_param_name)
{
  if (!estimator_cfg_.enabled || !center_msg || !side_msg) {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI> center_cloud;
  pcl::PointCloud<pcl::PointXYZI> side_cloud;
  pcl::fromROSMsg(*center_msg, center_cloud);
  pcl::fromROSMsg(*side_msg, side_cloud);

  if (
    static_cast<int>(center_cloud.size()) < estimator_cfg_.min_points ||
    static_cast<int>(side_cloud.size()) < estimator_cfg_.min_points)
  {
    return false;
  }

  const rclcpp::Time center_stamp(center_msg->header.stamp);
  const rclcpp::Time side_stamp(side_msg->header.stamp);

  double abs_gyro_z = 0.0;
  if (!get_abs_gyro_z_at_time(center_stamp, abs_gyro_z) || abs_gyro_z < estimator_cfg_.min_yaw_rate_rad_s) {
    return false;
  }

  const double start = std::max(
    side_time_offset_sec - estimator_cfg_.search_half_window_sec, estimator_cfg_.min_offset_sec);
  const double end = std::min(
    side_time_offset_sec + estimator_cfg_.search_half_window_sec, estimator_cfg_.max_offset_sec);

  double best_offset = side_time_offset_sec;
  double best_score = -std::numeric_limits<double>::infinity();

  for (double candidate = start; candidate <= end + 1e-9; candidate += estimator_cfg_.search_step_sec) {
    const double score = score_side_offset_candidate(
      center_cloud,
      side_cloud,
      center_stamp,
      side_stamp,
      t_center_side,
      candidate);

    if (score > best_score) {
      best_score = score;
      best_offset = candidate;
    }
  }

  if (!std::isfinite(best_score)) {
    return false;
  }

  const double score_gain = best_score - last_score_out;
  if (score_gain < estimator_cfg_.min_score_gain && last_score_out > 0.0) {
    return false;
  }

  const double old_offset = side_time_offset_sec;
  side_time_offset_sec =
    (1.0 - estimator_cfg_.ema_alpha) * side_time_offset_sec + estimator_cfg_.ema_alpha * best_offset;
  side_time_offset_sec = std::clamp(side_time_offset_sec, estimator_cfg_.min_offset_sec, estimator_cfg_.max_offset_sec);
  last_score_out = best_score;

  if (estimator_cfg_.write_back_to_params) {
    set_parameters({rclcpp::Parameter(side_param_name, side_time_offset_sec)});
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "Offset update [%s]: old=%.4f best=%.4f new=%.4f score=%.3f gyro_z=%.3f",
    side_name,
    old_offset,
    best_offset,
    side_time_offset_sec,
    best_score,
    abs_gyro_z);
  return true;
}

void LidarAggregatorNode::publish_offset_diagnostics_locked(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::Vector3Stamped offsets_msg;
  offsets_msg.header.stamp = stamp;
  offsets_msg.header.frame_id = center_frame_;
  offsets_msg.vector.x = ne_time_offset_sec_;
  offsets_msg.vector.y = nw_time_offset_sec_;
  offsets_msg.vector.z = 0.0;
  pub_estimated_offsets_->publish(offsets_msg);

  geometry_msgs::msg::Vector3Stamped score_msg;
  score_msg.header = offsets_msg.header;
  score_msg.vector.x = estimator_runtime_.ne_last_score;
  score_msg.vector.y = estimator_runtime_.nw_last_score;
  score_msg.vector.z = 0.0;
  pub_offset_scores_->publish(score_msg);
}

void LidarAggregatorNode::try_publish_fusion_locked(const sensor_msgs::msg::PointCloud2::SharedPtr & center_msg)
{
  if (!center_msg || !latest_ne_ || !latest_nw_) {
    return;
  }

  if (estimator_cfg_.enabled) {
    maybe_update_side_offset_locked(
      center_msg,
      latest_ne_,
      t_center_ne_,
      ne_time_offset_sec_,
      estimator_runtime_.ne_last_score,
      "ne",
      "timing.ne_time_offset_sec");
    maybe_update_side_offset_locked(
      center_msg,
      latest_nw_,
      t_center_nw_,
      nw_time_offset_sec_,
      estimator_runtime_.nw_last_score,
      "nw",
      "timing.nw_time_offset_sec");
  }

  const rclcpp::Time center_stamp(center_msg->header.stamp);
  publish_offset_diagnostics_locked(center_stamp);

  const rclcpp::Time ne_aligned =
    rclcpp::Time(latest_ne_->header.stamp) + rclcpp::Duration::from_seconds(ne_time_offset_sec_);
  const rclcpp::Time nw_aligned =
    rclcpp::Time(latest_nw_->header.stamp) + rclcpp::Duration::from_seconds(nw_time_offset_sec_);

  if (abs_time_delta_sec(ne_aligned, center_stamp) > max_pair_dt_sec_ ||
    abs_time_delta_sec(nw_aligned, center_stamp) > max_pair_dt_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Skipping fusion due to pair time mismatch. Adjust timing offsets or max_pair_dt_sec.");
    return;
  }

  auto ne_corrected = compensate_side_cloud(latest_ne_, center_stamp, t_center_ne_, ne_time_offset_sec_, center_frame_);
  auto nw_corrected = compensate_side_cloud(latest_nw_, center_stamp, t_center_nw_, nw_time_offset_sec_, center_frame_);

  if (!ne_corrected || !nw_corrected) {
    return;
  }

  auto center_in_center_frame = std::make_shared<sensor_msgs::msg::PointCloud2>(*center_msg);
  center_in_center_frame->header.stamp = center_stamp;
  center_in_center_frame->header.frame_id = center_frame_;

  pub_ne_deskewed_cc_->publish(*ne_corrected);
  pub_nw_deskewed_cc_->publish(*nw_corrected);

  auto merged = merge_clouds(center_in_center_frame, ne_corrected, nw_corrected, center_stamp, center_frame_);
  if (merged) {
    pub_merged_->publish(*merged);
  }
}

RigidTransform LidarAggregatorNode::rigid_from_xyz_rpy(
  double x,
  double y,
  double z,
  double roll,
  double pitch,
  double yaw)
{
  RigidTransform tf;
  Eigen::AngleAxisd roll_a(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_a(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_a(yaw, Eigen::Vector3d::UnitZ());
  tf.rotation = (yaw_a * pitch_a * roll_a).toRotationMatrix();
  tf.translation = Eigen::Vector3d(x, y, z);
  return tf;
}

}  // namespace lidar_aggregator

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_aggregator::LidarAggregatorNode)
