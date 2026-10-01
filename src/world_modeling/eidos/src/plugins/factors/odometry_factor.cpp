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

#include "eidos/plugins/factors/odometry_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <tf2/exceptions.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace eidos
{

namespace
{

gtsam::Pose3 toPose3(const geometry_msgs::msg::Pose & p)
{
  return gtsam::Pose3(
    gtsam::Rot3::Quaternion(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
    gtsam::Point3(p.position.x, p.position.y, p.position.z));
}

double toSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

constexpr double kBackwardsResetSec = 1.0;  // a stamp this far behind the newest = bag loop / clock reset
constexpr double kLogPeriodSec = 10.0;

}  // namespace

// ==========================================================================
// Lifecycle
// ==========================================================================

void OdometryFactor::onInitialize()
{
  const std::string & prefix = name_;

  node_->declare_parameter(prefix + ".odom_topic", std::string("/perception/visual_odometry/odometry"));
  node_->declare_parameter(prefix + ".time_offset", time_offset_);
  node_->declare_parameter(prefix + ".max_gap", max_gap_);
  node_->declare_parameter(prefix + ".max_pending_age", max_pending_age_);
  node_->declare_parameter(prefix + ".buffer_duration", buffer_duration_);
  node_->declare_parameter(prefix + ".rot_sigma", rot_sigma_);
  node_->declare_parameter(prefix + ".rot_sigma_per_m", rot_sigma_per_m_);
  node_->declare_parameter(prefix + ".trans_sigma", trans_sigma_);
  node_->declare_parameter(prefix + ".trans_sigma_per_m", trans_sigma_per_m_);
  node_->declare_parameter(prefix + ".robust_kernel", robust_kernel_);
  node_->declare_parameter(prefix + ".robust_k", robust_k_);
  node_->declare_parameter(prefix + ".gate_trans", gate_trans_);
  node_->declare_parameter(prefix + ".gate_rot", gate_rot_);
  node_->declare_parameter(prefix + ".add_factors", add_factors_);

  node_->get_parameter(prefix + ".odom_topic", odom_topic_);
  node_->get_parameter(prefix + ".time_offset", time_offset_);
  node_->get_parameter(prefix + ".max_gap", max_gap_);
  node_->get_parameter(prefix + ".max_pending_age", max_pending_age_);
  node_->get_parameter(prefix + ".buffer_duration", buffer_duration_);
  node_->get_parameter(prefix + ".rot_sigma", rot_sigma_);
  node_->get_parameter(prefix + ".rot_sigma_per_m", rot_sigma_per_m_);
  node_->get_parameter(prefix + ".trans_sigma", trans_sigma_);
  node_->get_parameter(prefix + ".trans_sigma_per_m", trans_sigma_per_m_);
  node_->get_parameter(prefix + ".robust_kernel", robust_kernel_);
  node_->get_parameter(prefix + ".robust_k", robust_k_);
  node_->get_parameter(prefix + ".gate_trans", gate_trans_);
  node_->get_parameter(prefix + ".gate_rot", gate_rot_);
  node_->get_parameter(prefix + ".add_factors", add_factors_);
  node_->get_parameter("frames.base_link", base_link_frame_);

  if (robust_kernel_ != "none" && robust_kernel_ != "huber" && robust_kernel_ != "cauchy") {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] unknown robust_kernel '%s' (none|huber|cauchy); using huber",
      name_.c_str(),
      robust_kernel_.c_str());
    robust_kernel_ = "huber";
  }
  buffer_.emplace(max_gap_, buffer_duration_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&OdometryFactor::odometryCallback, this, std::placeholders::_1),
    sub_opts);
  residual_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(name_ + "/residual", 10);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] initialized (odometry factor, topic=%s, add_factors=%d, time_offset=%.3f s, max_gap=%.3f s)",
    name_.c_str(),
    odom_topic_.c_str(),
    add_factors_,
    time_offset_,
    max_gap_);
}

void OdometryFactor::activate()
{
  active_ = true;
  residual_pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void OdometryFactor::deactivate()
{
  active_ = false;
  residual_pub_->on_deactivate();
  {
    std::lock_guard<std::mutex> lock(buffer_mtx_);
    buffer_->clear();
  }
  pairs_.clear();
  graph_poses_.clear();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

// ==========================================================================
// Odometry input
// ==========================================================================

bool OdometryFactor::resolveExtrinsic(const std::string & child_frame)
{
  if (child_frame.empty() || child_frame == base_link_frame_) {
    T_base_child_ = gtsam::Pose3();
    return true;
  }
  try {
    auto tf_msg = tf_->lookupTransform(base_link_frame_, child_frame, tf2::TimePointZero);
    const auto & t = tf_msg.transform.translation;
    const auto & r = tf_msg.transform.rotation;
    T_base_child_ = gtsam::Pose3(gtsam::Rot3::Quaternion(r.w, r.x, r.y, r.z), gtsam::Point3(t.x, t.y, t.z));
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] %s <- %s: t=(%.3f, %.3f, %.3f)",
      name_.c_str(),
      base_link_frame_.c_str(),
      child_frame.c_str(),
      t.x,
      t.y,
      t.z);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "[%s] waiting for TF %s <- %s: %s",
      name_.c_str(),
      base_link_frame_.c_str(),
      child_frame.c_str(),
      ex.what());
    return false;
  }
}

void OdometryFactor::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!active_) return;
  std::lock_guard<std::mutex> lock(buffer_mtx_);
  if (!T_base_child_) {
    if (!resolveExtrinsic(msg->child_frame_id)) return;
    source_frame_ = msg->header.frame_id;
  }
  if (msg->header.frame_id != source_frame_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "[%s] ignoring odometry in frame '%s' (expected '%s')",
      name_.c_str(),
      msg->header.frame_id.c_str(),
      source_frame_.c_str());
    return;
  }
  const double t = toSec(msg->header.stamp) + time_offset_;
  const gtsam::Pose3 pose = toPose3(msg->pose.pose);
  if (!buffer_->push(t, pose) && buffer_->newest() - t > kBackwardsResetSec) {
    buffer_->clear();  // time jumped backwards (bag loop / clock reset)
    buffer_->push(t, pose);
  }
}

// ==========================================================================
// Factors
// ==========================================================================

gtsam::SharedNoiseModel OdometryFactor::noiseFor(double distance) const
{
  const double sr = rot_sigma_ + rot_sigma_per_m_ * distance;
  const double st = trans_sigma_ + trans_sigma_per_m_ * distance;
  // GTSAM Pose3 tangent order: [rot, trans]
  auto diagonal = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector6() << sr, sr, sr, st, st, st).finished());
  if (robust_kernel_ == "huber") {
    return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(robust_k_), diagonal);
  }
  if (robust_kernel_ == "cauchy") {
    return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(robust_k_), diagonal);
  }
  return diagonal;
}

StampedFactorResult OdometryFactor::latchFactor(gtsam::Key key, double timestamp)
{
  StampedFactorResult result;
  if (!active_) return result;
  if (state_->load(std::memory_order_acquire) != SlamState::TRACKING) return result;

  pairs_.addState(key, timestamp);

  std::lock_guard<std::mutex> lock(buffer_mtx_);
  const bool have_odometry = !buffer_->empty() && T_base_child_.has_value();
  const bool gating = gate_trans_ > 0.0 || gate_rot_ > 0.0;
  while (!pairs_.empty()) {
    const KeyframePairQueue::Pair pair = pairs_.front();
    if (!have_odometry || pair.to.t > buffer_->newest()) {
      // Odometry does not reach this pair yet (it lags the keyframe sensor); give up after a while.
      if (timestamp - pair.to.t <= max_pending_age_) break;
      ++counters_.expired;
      pairs_.pop();
      continue;
    }
    auto from_it = graph_poses_.find(pair.from.key);
    auto to_it = graph_poses_.find(pair.to.key);
    const bool have_graph = from_it != graph_poses_.end() && to_it != graph_poses_.end();
    // The newest state has no optimized estimate yet: wait one optimization to be able to gate it.
    if (gating && !have_graph && pair.to.key == key) break;
    pairs_.pop();

    gtsam::Pose3 rel_child;
    const auto status = buffer_->relative(pair.from.t, pair.to.t, rel_child);
    if (status == OdometryBuffer::Status::kGap) {
      ++counters_.gaps;
      continue;
    }
    if (status != OdometryBuffer::Status::kOk) {
      ++counters_.out_of_range;
      continue;
    }
    const gtsam::Pose3 rel = conjugateMotion(*T_base_child_, rel_child);

    if (have_graph) {
      const gtsam::Pose3 residual = rel.between(from_it->second.between(to_it->second));
      const double res_trans = residual.translation().norm();
      const double res_rot = gtsam::Rot3::Logmap(residual.rotation()).norm();

      geometry_msgs::msg::PoseStamped res_msg;
      res_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(pair.to.t * 1e9), RCL_ROS_TIME);
      res_msg.header.frame_id = base_link_frame_;
      const auto q = residual.rotation().toQuaternion();
      res_msg.pose.position.x = residual.x();
      res_msg.pose.position.y = residual.y();
      res_msg.pose.position.z = residual.z();
      res_msg.pose.orientation.w = q.w();
      res_msg.pose.orientation.x = q.x();
      res_msg.pose.orientation.y = q.y();
      res_msg.pose.orientation.z = q.z();
      residual_pub_->publish(res_msg);

      if ((gate_trans_ > 0.0 && res_trans > gate_trans_) || (gate_rot_ > 0.0 && res_rot > gate_rot_)) {
        ++counters_.gated;
        gtsam::Symbol from_sym(pair.from.key), to_sym(pair.to.key);
        RCLCPP_WARN(
          node_->get_logger(),
          "[%s] gated (%c,%lu)->(%c,%lu): disagrees with graph by %.2f m / %.2f deg",
          name_.c_str(),
          from_sym.chr(),
          from_sym.index(),
          to_sym.chr(),
          to_sym.index(),
          res_trans,
          res_rot * 180.0 / M_PI);
        continue;
      }
    } else {
      ++counters_.ungated;
    }

    ++counters_.emitted;
    if (add_factors_) {
      result.factors.push_back(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        pair.from.key, pair.to.key, rel, noiseFor(rel.translation().norm())));
    }
  }
  logCounters();
  return result;
}

void OdometryFactor::onOptimizationComplete(const gtsam::Values & values, bool /*graph_corrected*/)
{
  if (!active_) return;
  graph_poses_.clear();
  for (const auto & pair : pairs_.pending()) {
    for (gtsam::Key k : {pair.from.key, pair.to.key}) {
      if (values.exists(k)) graph_poses_[k] = values.at<gtsam::Pose3>(k);
    }
  }
}

void OdometryFactor::logCounters()
{
  const rclcpp::Time now = node_->get_clock()->now();
  if (last_log_time_.nanoseconds() != 0 && (now - last_log_time_).seconds() < kLogPeriodSec) return;
  last_log_time_ = now;
  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] %s=%zu gated=%zu ungated=%zu gap=%zu out_of_range=%zu expired=%zu pending=%zu",
    name_.c_str(),
    add_factors_ ? "factors" : "shadow",
    counters_.emitted,
    counters_.gated,
    counters_.ungated,
    counters_.gaps,
    counters_.out_of_range,
    counters_.expired,
    pairs_.size());
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::OdometryFactor, eidos::FactorPlugin)
