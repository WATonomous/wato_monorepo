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

#include "eidos/plugins/visual/lidar_depth.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

#include "eidos/utils/lidar_deskew.hpp"

namespace eidos
{

namespace
{

/// A twist further than this from an image's time is not used for it (s).
constexpr double kMaxTwistGap = 0.1;

double toSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

std::optional<std::array<uint32_t, 3>> xyzOffsets(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::array<int64_t, 3> found{-1, -1, -1};
  for (const auto & f : cloud.fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
    if (f.name == "x") found[0] = f.offset;
    if (f.name == "y") found[1] = f.offset;
    if (f.name == "z") found[2] = f.offset;
  }
  if (found[0] < 0 || found[1] < 0 || found[2] < 0) return std::nullopt;
  return std::array<uint32_t, 3>{
    static_cast<uint32_t>(found[0]), static_cast<uint32_t>(found[1]), static_cast<uint32_t>(found[2])};
}

float readFloat(const uint8_t * p)
{
  float v;
  std::memcpy(&v, p, sizeof(float));
  return v;
}

sensor_msgs::msg::PointCloud2 makeCloud(const std::string & frame, double stamp, const std::vector<float> & xyz)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9), RCL_ROS_TIME);
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(xyz.size() / 3);
  const char * names[] = {"x", "y", "z"};
  for (uint32_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = 4 * i;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    cloud.fields.push_back(f);
  }
  cloud.is_bigendian = false;
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_dense = true;
  cloud.data.resize(xyz.size() * sizeof(float));
  std::memcpy(cloud.data.data(), xyz.data(), cloud.data.size());
  return cloud;
}

}  // namespace

LidarDepth::LidarDepth(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & prefix,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  tf2_ros::Buffer * tf)
: node_(node)
, name_(prefix)
, tf_(tf)
{
  const std::string p = prefix + ".depth";
  std::string lidar_topic, odometry_topic;
  node_->declare_parameter(p + ".lidar_topic", std::string("/lidar_cc/velodyne_points"));
  node_->declare_parameter(p + ".odometry_topic", std::string("imu_factor/odometry"));
  node_->declare_parameter(p + ".sweep_period", sweep_period_);
  node_->declare_parameter(p + ".window", window_);
  node_->declare_parameter(p + ".min_range", min_range_);
  node_->declare_parameter(p + ".max_range", max_range_);
  node_->declare_parameter(p + ".buffer_duration", buffer_duration_);
  node_->get_parameter(p + ".lidar_topic", lidar_topic);
  node_->get_parameter(p + ".odometry_topic", odometry_topic);
  node_->get_parameter(p + ".sweep_period", sweep_period_);
  node_->get_parameter(p + ".window", window_);
  node_->get_parameter(p + ".min_range", min_range_);
  node_->get_parameter(p + ".max_range", max_range_);
  node_->get_parameter(p + ".buffer_duration", buffer_duration_);
  node_->get_parameter("frames.base_link", base_frame_);

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = callback_group;
  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { lidarCallback(msg); },
    opts);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic,
    rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odometryCallback(msg); },
    opts);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] lidar depth from %s (sweep %.3f s, window %.3f s, %.1f-%.1f m), deskewed with the twist on %s",
    name_.c_str(),
    lidar_topic.c_str(),
    sweep_period_,
    window_,
    min_range_,
    max_range_,
    odometry_topic.c_str());
}

void LidarDepth::setActive(bool active)
{
  std::lock_guard<std::mutex> lock(mtx_);
  active_ = active;
  if (!active) {
    scans_.clear();
    twists_.clear();
  }
}

void LidarDepth::lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const double t = toSec(msg->header.stamp);
  std::lock_guard<std::mutex> lock(mtx_);
  if (!active_) return;
  if (!scans_.empty() && t <= scans_.back().first) {
    if (t < scans_.back().first - 1.0) scans_.clear();  // the clock restarted (bag replayed)
    else return;
  }
  scans_.emplace_back(t, msg);
  while (scans_.size() > 1 && t - scans_.front().first > buffer_duration_) scans_.pop_front();
}

void LidarDepth::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const double t = toSec(msg->header.stamp);
  const auto & l = msg->twist.twist.linear;
  const auto & a = msg->twist.twist.angular;
  std::lock_guard<std::mutex> lock(mtx_);
  if (!active_) return;
  if (!twists_.empty() && t <= twists_.back().t) {
    if (t < twists_.back().t - 1.0) twists_.clear();
    else return;
  }
  twists_.push_back({t, Eigen::Vector3d(l.x, l.y, l.z), Eigen::Vector3d(a.x, a.y, a.z)});
  while (twists_.size() > 1 && t - twists_.front().t > buffer_duration_) twists_.pop_front();
}

bool LidarDepth::covers(double t)
{
  std::lock_guard<std::mutex> lock(mtx_);
  return !scans_.empty() && scans_.back().first >= t + window_;
}

std::optional<Eigen::Isometry3d> LidarDepth::lookup(const std::string & target, const std::string & source)
{
  const auto key = std::make_pair(target, source);
  const auto it = extrinsics_.find(key);
  if (it != extrinsics_.end()) return it->second;
  try {
    const auto tf = tf_->lookupTransform(target, source, tf2::TimePointZero);
    const auto & r = tf.transform.rotation;
    const auto & p = tf.transform.translation;
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized().toRotationMatrix();
    t.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    extrinsics_[key] = t;
    return t;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "[%s] no transform %s <- %s: %s", name_.c_str(),
      target.c_str(), source.c_str(), e.what());
    return std::nullopt;
  }
}

std::vector<sensor_msgs::msg::PointCloud2> LidarDepth::clouds(
  const VisualFrameSet & set, double time_offset, Stats & stats)
{
  std::vector<std::pair<double, sensor_msgs::msg::PointCloud2::ConstSharedPtr>> scans;
  std::deque<Twist> twists;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    scans.assign(scans_.begin(), scans_.end());
    twists = twists_;
  }

  std::vector<sensor_msgs::msg::PointCloud2> out;
  out.reserve(set.frames.size());
  for (const auto & frame : set.frames) {
    const double tau = frame->stamp + time_offset;
    std::vector<float> xyz;
    const auto cam_from_base = lookup(frame->frame_id, base_frame_);

    // Body twist nearest the image's time
    const Twist * twist = nullptr;
    double best = kMaxTwistGap;
    for (const auto & tw : twists) {
      if (std::abs(tw.t - tau) < best) {
        best = std::abs(tw.t - tau);
        twist = &tw;
      }
    }
    if (!twist) stats.deskewed = false;
    const Eigen::Vector3d v = twist ? twist->v : Eigen::Vector3d::Zero();
    const Eigen::Vector3d w = twist ? twist->w : Eigen::Vector3d::Zero();

    for (const auto & [stamp, cloud] : scans) {
      if (!cam_from_base) break;
      if (stamp < tau - window_ || stamp - sweep_period_ > tau + window_) continue;
      const auto offsets = xyzOffsets(*cloud);
      const auto base_from_lidar = lookup(base_frame_, cloud->header.frame_id);
      if (!offsets || !base_from_lidar) continue;
      const std::size_t rows = cloud->height;
      for (std::size_t row = 0; row < rows; ++row) {
        const double t = firingTime(stamp, sweep_period_, row, rows);
        if (std::abs(t - tau) >= window_) continue;
        for (std::size_t col = 0; col < cloud->width; ++col) {
          const uint8_t * p = cloud->data.data() + row * cloud->row_step + col * cloud->point_step;
          const Eigen::Vector3d lidar(readFloat(p + (*offsets)[0]), readFloat(p + (*offsets)[1]), readFloat(p + (*offsets)[2]));
          if (!lidar.allFinite()) continue;
          const double range = lidar.norm();
          if (range < min_range_ || range > max_range_) continue;
          const Eigen::Vector3d cam = *cam_from_base * moveToTime(*base_from_lidar * lidar, t - tau, v, w);
          if (cam.z() <= min_range_) continue;
          xyz.push_back(static_cast<float>(cam.x()));
          xyz.push_back(static_cast<float>(cam.y()));
          xyz.push_back(static_cast<float>(cam.z()));
        }
      }
    }
    stats.points += xyz.size() / 3;
    if (xyz.empty()) ++stats.images_without_points;
    out.push_back(makeCloud(frame->frame_id, tau, xyz));
  }
  return out;
}

}  // namespace eidos
