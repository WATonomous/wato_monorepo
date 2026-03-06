// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "lidar_aggregator/lidar_aggregator_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_aggregator
{

namespace
{

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

  declare_parameter<std::string>("frames.center", "lidar_cc");
  declare_parameter<int>("runtime.qos_depth", 20);
  declare_parameter<double>("runtime.max_pair_dt_sec", 0.08);
  declare_parameter<double>("runtime.max_imu_buffer_sec", 20.0);
  declare_parameter<double>("runtime.max_imu_interp_gap_sec", 0.05);

  declare_parameter<double>("timing.ne_time_offset_sec", 0.0);
  declare_parameter<double>("timing.nw_time_offset_sec", 0.0);

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

  get_parameter("frames.center", center_frame_);
  get_parameter("runtime.qos_depth", qos_depth_);
  get_parameter("runtime.max_pair_dt_sec", max_pair_dt_sec_);
  get_parameter("runtime.max_imu_buffer_sec", max_imu_buffer_sec_);
  get_parameter("runtime.max_imu_interp_gap_sec", max_imu_interp_gap_sec_);
  get_parameter("timing.ne_time_offset_sec", ne_time_offset_sec_);
  get_parameter("timing.nw_time_offset_sec", nw_time_offset_sec_);

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

  imu_buffer_.push_back(ImuSample{rclcpp::Time(msg->header.stamp), q});

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

  const rclcpp::Time side_stamp = rclcpp::Time(side_msg->header.stamp) + rclcpp::Duration::from_seconds(side_time_offset_sec);
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

void LidarAggregatorNode::try_publish_fusion_locked(const sensor_msgs::msg::PointCloud2::SharedPtr & center_msg)
{
  if (!center_msg || !latest_ne_ || !latest_nw_) {
    return;
  }

  const rclcpp::Time center_stamp(center_msg->header.stamp);

  const rclcpp::Time ne_aligned = rclcpp::Time(latest_ne_->header.stamp) + rclcpp::Duration::from_seconds(ne_time_offset_sec_);
  const rclcpp::Time nw_aligned = rclcpp::Time(latest_nw_->header.stamp) + rclcpp::Duration::from_seconds(nw_time_offset_sec_);

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
