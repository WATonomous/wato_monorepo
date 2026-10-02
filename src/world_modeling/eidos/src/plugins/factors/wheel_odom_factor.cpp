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

#include "eidos/plugins/factors/wheel_odom_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace eidos
{

namespace wk = wheel_kinematics;

// Pose covariance on the published odometry: the dead-reckoned pose is for
// visualization / debugging only and is not meant to be fused.
static constexpr double kUnfusedPoseVariance = 1.0e6;

// ==========================================================================
// Lifecycle
// ==========================================================================

void WheelOdomFactor::onInitialize()
{
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".joint_states_topic", std::string("wheel_joint_states"));
  node_->declare_parameter(prefix + ".left_joint", std::string("rear_left_joint"));
  node_->declare_parameter(prefix + ".right_joint", std::string("rear_right_joint"));
  node_->declare_parameter(prefix + ".wheel_radius", wheel_radius_);
  node_->declare_parameter(prefix + ".axle_frame", std::string("rear_axle"));
  node_->declare_parameter(prefix + ".imu_topic", std::string("/imu/data"));
  node_->declare_parameter(prefix + ".imu_frame", std::string("imu_link"));
  node_->declare_parameter(prefix + ".odom_topic", std::string(name_ + "/odometry"));
  node_->declare_parameter(prefix + ".add_factors", add_factors_);
  node_->declare_parameter(prefix + ".max_dt", max_dt_);
  node_->declare_parameter(prefix + ".speed_std_moving", speed_noise_.speed_std_moving);
  node_->declare_parameter(prefix + ".zero_speed_std", speed_noise_.zero_speed_std);
  node_->declare_parameter(prefix + ".lateral_std", speed_noise_.lateral_std);
  node_->declare_parameter(prefix + ".vertical_std", speed_noise_.vertical_std);
  node_->declare_parameter(prefix + ".yaw_rate_std", speed_noise_.yaw_rate_std);
  node_->declare_parameter(prefix + ".factor.trans_std_per_meter", factor_noise_.trans_std_per_meter);
  node_->declare_parameter(prefix + ".factor.yaw_std_per_rad", factor_noise_.yaw_std_per_rad);
  node_->declare_parameter(prefix + ".factor.min_std", factor_noise_.min_std);
  node_->declare_parameter(prefix + ".factor.loose_std", factor_noise_.loose_std);

  std::string odom_topic;
  node_->get_parameter(prefix + ".joint_states_topic", joint_states_topic_);
  node_->get_parameter(prefix + ".left_joint", left_joint_);
  node_->get_parameter(prefix + ".right_joint", right_joint_);
  node_->get_parameter(prefix + ".wheel_radius", wheel_radius_);
  node_->get_parameter(prefix + ".axle_frame", axle_frame_);
  node_->get_parameter(prefix + ".imu_topic", imu_topic_);
  node_->get_parameter(prefix + ".imu_frame", imu_frame_);
  node_->get_parameter(prefix + ".odom_topic", odom_topic);
  node_->get_parameter(prefix + ".add_factors", add_factors_);
  node_->get_parameter(prefix + ".max_dt", max_dt_);
  node_->get_parameter(prefix + ".speed_std_moving", speed_noise_.speed_std_moving);
  node_->get_parameter(prefix + ".zero_speed_std", speed_noise_.zero_speed_std);
  node_->get_parameter(prefix + ".lateral_std", speed_noise_.lateral_std);
  node_->get_parameter(prefix + ".vertical_std", speed_noise_.vertical_std);
  node_->get_parameter(prefix + ".yaw_rate_std", speed_noise_.yaw_rate_std);
  node_->get_parameter(prefix + ".factor.trans_std_per_meter", factor_noise_.trans_std_per_meter);
  node_->get_parameter(prefix + ".factor.yaw_std_per_rad", factor_noise_.yaw_std_per_rad);
  node_->get_parameter(prefix + ".factor.min_std", factor_noise_.min_std);
  node_->get_parameter(prefix + ".factor.loose_std", factor_noise_.loose_std);
  factor_noise_.zero_speed_std = speed_noise_.zero_speed_std;
  node_->get_parameter("frames.odometry", odom_frame_);
  node_->get_parameter("frames.base_link", base_link_frame_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;
  joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&WheelOdomFactor::jointStateCallback, this, std::placeholders::_1),
    sub_opts);
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&WheelOdomFactor::imuCallback, this, std::placeholders::_1),
    sub_opts);

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] initialized (wheel odom factor, add_factors=%d, joints=%s/%s, odom=%s)",
    name_.c_str(),
    add_factors_,
    left_joint_.c_str(),
    right_joint_.c_str(),
    odom_topic.c_str());
}

void WheelOdomFactor::activate()
{
  active_ = true;
  odom_pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void WheelOdomFactor::deactivate()
{
  active_ = false;
  odom_pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

// ==========================================================================
// latchFactor — planar BetweenFactor between previous and current state
// ==========================================================================

StampedFactorResult WheelOdomFactor::latchFactor(gtsam::Key key, double timestamp)
{
  StampedFactorResult result;
  if (!active_ || !add_factors_) return result;
  if (!has_last_key_) {
    last_key_ = key;
    last_key_time_ = timestamp;
    has_last_key_ = true;
    return result;
  }

  wk::Segment seg;
  bool has_samples = false;
  {
    std::lock_guard lock(sample_mtx_);
    has_samples = std::any_of(
      samples_.begin(), samples_.end(), [&](const wk::Sample & s) { return s.t > last_key_time_ && s.t <= timestamp; });
    if (has_samples) {
      seg = wk::integrateSegment(samples_, last_key_time_, timestamp, max_dt_);
    }
    while (!samples_.empty() && samples_.front().t <= timestamp) samples_.pop_front();
  }

  // No wheel data over the interval: skip rather than assert "did not move".
  if (has_samples) {
    gtsam::Pose3 delta(gtsam::Rot3::Yaw(seg.delta.theta), gtsam::Point3(seg.delta.x, seg.delta.y, 0.0));
    auto stds = wk::betweenStd(seg, factor_noise_);
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << stds[0], stds[1], stds[2], stds[3], stds[4], stds[5]).finished());
    result.factors.push_back(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(last_key_, key, delta, noise));

    gtsam::Symbol prev_x(last_key_);
    gtsam::Symbol curr_x(key);
    RCLCPP_INFO(
      node_->get_logger(),
      "\033[34m[%s] WheelBetween (%c,%lu)->(%c,%lu) dist=%.2f yaw=%.3f\033[0m",
      name_.c_str(),
      prev_x.chr(),
      prev_x.index(),
      curr_x.chr(),
      curr_x.index(),
      seg.distance,
      seg.delta.theta);
  }

  last_key_ = key;
  last_key_time_ = timestamp;
  return result;
}

// ==========================================================================
// Callbacks
// ==========================================================================

bool WheelOdomFactor::resolveExtrinsics()
{
  if (!has_imu_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, imu_frame_, tf2::TimePointZero);
      const auto & r = tf_msg.transform.rotation;
      R_base_imu_ = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      has_imu_tf_ = true;
    } catch (const tf2::TransformException &) {
    }
  }
  if (!has_axle_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, axle_frame_, tf2::TimePointZero);
      const auto & t = tf_msg.transform.translation;
      p_base_axle_ = Eigen::Vector3d(t.x, t.y, t.z);
      has_axle_tf_ = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] axle lever arm %s -> %s: [%.3f, %.3f, %.3f]",
        name_.c_str(),
        base_link_frame_.c_str(),
        axle_frame_.c_str(),
        t.x,
        t.y,
        t.z);
    } catch (const tf2::TransformException &) {
    }
  }
  return has_imu_tf_ && has_axle_tf_;
}

void WheelOdomFactor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!active_ || !has_imu_tf_) return;

  Eigen::Vector3d gyr(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  std::lock_guard lock(imu_mtx_);
  latest_gyro_base_ = R_base_imu_ * gyr;
  has_gyro_ = true;
}

void WheelOdomFactor::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!active_) return;

  if (!resolveExtrinsics()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "[%s] waiting for TF (%s -> %s, %s -> %s)",
      name_.c_str(),
      base_link_frame_.c_str(),
      imu_frame_.c_str(),
      base_link_frame_.c_str(),
      axle_frame_.c_str());
    return;
  }

  // Find the left/right wheel joint velocities
  double w_left = 0.0;
  double w_right = 0.0;
  bool found_left = false;
  bool found_right = false;
  for (size_t i = 0; i < msg->name.size() && i < msg->velocity.size(); ++i) {
    if (msg->name[i] == left_joint_) {
      w_left = msg->velocity[i];
      found_left = true;
    } else if (msg->name[i] == right_joint_) {
      w_right = msg->velocity[i];
      found_right = true;
    }
  }
  if (!found_left || !found_right) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "[%s] joint state is missing velocity for %s and/or %s",
      name_.c_str(),
      left_joint_.c_str(),
      right_joint_.c_str());
    return;
  }

  Eigen::Vector3d gyro;
  {
    std::lock_guard lock(imu_mtx_);
    if (!has_gyro_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        5000,
        "[%s] waiting for IMU on %s",
        name_.c_str(),
        imu_topic_.c_str());
      return;
    }
    gyro = latest_gyro_base_;
  }

  // Rear-axle speed -> base-frame velocity via the axle lever arm. Yaw rate only:
  // roll/pitch rates would add vertical velocity the wheels do not measure.
  const double v_axle = wk::axleSpeed(w_left, w_right, wheel_radius_);
  const Eigen::Vector3d omega(0.0, 0.0, gyro.z());
  const Eigen::Vector3d v_base = wk::axleToBaseVelocity(Eigen::Vector3d(v_axle, 0.0, 0.0), omega, p_base_axle_);

  wk::Sample sample;
  sample.t = rclcpp::Time(msg->header.stamp).seconds();
  sample.vx = v_base.x();
  sample.vy = v_base.y();
  sample.wz = gyro.z();
  sample.zero_speed = v_axle <= 0.0;

  if (add_factors_) {
    std::lock_guard lock(sample_mtx_);
    samples_.push_back(sample);
    while (samples_.size() > kMaxSampleBuffer) samples_.pop_front();
  }

  // Live dead-reckoned pose (debug / setOdomPose only)
  if (last_sample_time_ > 0.0) {
    const double dt = sample.t - last_sample_time_;
    if (dt > 0.0 && dt <= max_dt_) {
      odom_pose_ = wk::compose(odom_pose_, wk::integrateConstantTwist(sample.vx, sample.vy, sample.wz, dt));
    }
  }
  last_sample_time_ = sample.t;
  setOdomPose(gtsam::Pose3(gtsam::Rot3::Yaw(odom_pose_.theta), gtsam::Point3(odom_pose_.x, odom_pose_.y, 0.0)));

  publishOdometry(msg->header.stamp, sample);
}

void WheelOdomFactor::publishOdometry(const builtin_interfaces::msg::Time & stamp, const wk::Sample & sample)
{
  if (!odom_pub_->is_activated()) return;

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_link_frame_;
  odom_msg.pose.pose.position.x = odom_pose_.x;
  odom_msg.pose.pose.position.y = odom_pose_.y;
  odom_msg.pose.pose.orientation.z = std::sin(odom_pose_.theta / 2.0);
  odom_msg.pose.pose.orientation.w = std::cos(odom_pose_.theta / 2.0);
  for (size_t i = 0; i < 6; ++i) {
    odom_msg.pose.covariance[i * 6 + i] = kUnfusedPoseVariance;
  }

  odom_msg.twist.twist.linear.x = sample.vx;
  odom_msg.twist.twist.linear.y = sample.vy;
  odom_msg.twist.twist.angular.z = sample.wz;
  const auto cov = wk::twistCovariance(sample.zero_speed, speed_noise_);
  std::copy(cov.begin(), cov.end(), odom_msg.twist.covariance.begin());

  odom_pub_->publish(odom_msg);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::WheelOdomFactor, eidos::FactorPlugin)
