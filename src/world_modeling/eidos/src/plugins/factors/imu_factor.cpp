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

#include "eidos/plugins/factors/imu_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace eidos
{

namespace
{

/// A stationary vehicle's accelerometer must read gravity within this fraction to level from it.
constexpr double kGravityTolerance = 0.1;
/// The first state counts as at rest if the gyro stayed below the threshold this long before it (s).
constexpr double kStationaryWindow = 0.5;
/// Initial uncertainty of the bias inside the preintegration (gtsam's biasAccOmegaInt).
constexpr double kBiasIntegrationCov = 1e-5;

double toSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

Eigen::Matrix3d diagonal(const std::vector<double> & v, const std::string & name)
{
  if (v.size() < 3) throw std::invalid_argument(name + " needs three values");
  return Eigen::Vector3d(v[0], v[1], v[2]).asDiagonal();
}

}  // namespace

gtsam::Key ImuFactor::velocityKey(gtsam::Key state)
{
  return gtsam::Symbol('v', gtsam::Symbol(state).index());
}

gtsam::Key ImuFactor::biasKey(gtsam::Key state)
{
  return gtsam::Symbol('b', gtsam::Symbol(state).index());
}

// ==========================================================================
// Lifecycle
// ==========================================================================

void ImuFactor::onInitialize()
{
  const std::string prefix = name_;

  node_->declare_parameter(prefix + ".imu_topic", std::string("/imu/data"));
  node_->declare_parameter(prefix + ".imu_frame", std::string("imu_link"));
  node_->declare_parameter(prefix + ".axes_rotation_rpy", std::vector<double>{0.0, 0.0, 0.0});
  node_->declare_parameter(prefix + ".add_factors", true);
  node_->declare_parameter(prefix + ".odom_topic", std::string(name_ + "/odometry"));
  node_->declare_parameter(prefix + ".acc_cov", std::vector<double>{9e-6, 9e-6, 9e-6});
  node_->declare_parameter(prefix + ".gyr_cov", std::vector<double>{1e-6, 1e-6, 1e-6});
  node_->declare_parameter(prefix + ".integration_cov", std::vector<double>{1e-8, 1e-8, 1e-8});
  node_->declare_parameter(prefix + ".acc_bias_cov", std::vector<double>{1e-6, 1e-6, 1e-6});
  node_->declare_parameter(prefix + ".gyr_bias_cov", std::vector<double>{4e-10, 4e-10, 4e-10});
  node_->declare_parameter(prefix + ".gravity", gravity_);
  node_->declare_parameter(prefix + ".initialization.warmup_samples", warmup_samples_);
  node_->declare_parameter(prefix + ".initialization.stationary_gyr_threshold", stationary_gyr_threshold_);
  node_->declare_parameter(prefix + ".prior.velocity_sigma_stationary", velocity_sigma_stationary_);
  node_->declare_parameter(prefix + ".prior.velocity_sigma_moving", velocity_sigma_moving_);
  node_->declare_parameter(prefix + ".prior.acc_bias_sigma", acc_bias_sigma_);
  node_->declare_parameter(prefix + ".prior.gyr_bias_sigma", gyr_bias_sigma_);

  bool add_factors = true;
  std::string odom_topic;
  node_->get_parameter(prefix + ".add_factors", add_factors);
  node_->get_parameter(prefix + ".odom_topic", odom_topic);
  node_->get_parameter(prefix + ".imu_topic", imu_topic_);
  node_->get_parameter(prefix + ".imu_frame", imu_frame_);
  node_->get_parameter(prefix + ".axes_rotation_rpy", axes_rotation_rpy_);
  node_->get_parameter(prefix + ".acc_cov", acc_cov_);
  node_->get_parameter(prefix + ".gyr_cov", gyr_cov_);
  node_->get_parameter(prefix + ".integration_cov", integration_cov_);
  node_->get_parameter(prefix + ".acc_bias_cov", acc_bias_cov_);
  node_->get_parameter(prefix + ".gyr_bias_cov", gyr_bias_cov_);
  node_->get_parameter(prefix + ".gravity", gravity_);
  node_->get_parameter(prefix + ".initialization.warmup_samples", warmup_samples_);
  node_->get_parameter(prefix + ".initialization.stationary_gyr_threshold", stationary_gyr_threshold_);
  node_->get_parameter(prefix + ".prior.velocity_sigma_stationary", velocity_sigma_stationary_);
  node_->get_parameter(prefix + ".prior.velocity_sigma_moving", velocity_sigma_moving_);
  node_->get_parameter(prefix + ".prior.acc_bias_sigma", acc_bias_sigma_);
  node_->get_parameter(prefix + ".prior.gyr_bias_sigma", gyr_bias_sigma_);
  node_->get_parameter("frames.odometry", odom_frame_);
  node_->get_parameter("frames.base_link", base_link_frame_);
  add_factors_ = add_factors;
  if (axes_rotation_rpy_.size() != 3) throw std::invalid_argument(name_ + ": axes_rotation_rpy needs three values");
  const double deg = M_PI / 180.0;
  imu_R_msg_ =
    gtsam::Rot3::RzRyRx(axes_rotation_rpy_[0] * deg, axes_rotation_rpy_[1] * deg, axes_rotation_rpy_[2] * deg);

  // Continuous-time noise of the IMU (gtsam's convention: covariance = noise density squared).
  params_ = Params::MakeSharedU(gravity_);
  params_->accelerometerCovariance = diagonal(acc_cov_, "acc_cov");
  params_->gyroscopeCovariance = diagonal(gyr_cov_, "gyr_cov");
  params_->integrationCovariance = diagonal(integration_cov_, "integration_cov");
  params_->biasAccCovariance = diagonal(acc_bias_cov_, "acc_bias_cov");
  params_->biasOmegaCovariance = diagonal(gyr_bias_cov_, "gyr_bias_cov");
  params_->biasAccOmegaInt = kBiasIntegrationCov * gtsam::I_6x6;

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(), std::bind(&ImuFactor::imuCallback, this, std::placeholders::_1), sub_opts);

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] initialized (IMU preintegration on %s, axes rotated by rpy (%.1f, %.1f, %.1f) deg into %s, "
    "add_factors=%d, odom=%s)",
    name_.c_str(),
    imu_topic_.c_str(),
    axes_rotation_rpy_[0],
    axes_rotation_rpy_[1],
    axes_rotation_rpy_[2],
    imu_frame_.c_str(),
    add_factors,
    odom_topic.c_str());
}

void ImuFactor::activate()
{
  active_ = true;
  odom_pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void ImuFactor::deactivate()
{
  active_ = false;
  odom_pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void ImuFactor::onTrackingBegin(const gtsam::Pose3 & pose)
{
  tracking_start_ = pose;
  has_last_key_ = false;
}

// ==========================================================================
// latchFactor — IMU factor between the previous state and this one
// ==========================================================================

StampedFactorResult ImuFactor::latchFactor(gtsam::Key key, double timestamp)
{
  StampedFactorResult result;
  if (!active_ || !add_factors_ || !isReady()) return result;
  const gtsam::Key v_key = velocityKey(key);
  const gtsam::Key b_key = biasKey(key);

  if (!has_last_key_) {
    // First state: gravity-aligned at the pose tracking started from, velocity and bias from the
    // warmup. The vehicle may already be moving, in which case the velocity prior is loose.
    bool still = false;
    {
      std::lock_guard<std::mutex> lock(imu_mtx_);
      still = stationaryBefore(timestamp, kStationaryWindow);
      while (!buffer_.empty() && buffer_.front().t <= timestamp) buffer_.pop_front();
    }
    const gtsam::Pose3 pose(
      gtsam::Rot3::Ypr(tracking_start_.rotation().yaw(), gravity_rotation_.pitch(), gravity_rotation_.roll()),
      tracking_start_.translation());
    const double v_sigma = still ? velocity_sigma_stationary_ : velocity_sigma_moving_;
    result.values.insert(key, pose);
    result.values.insert(v_key, gtsam::Vector3(gtsam::Vector3::Zero()));
    result.values.insert(b_key, initial_bias_);
    result.factors.push_back(gtsam::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
      v_key, gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, v_sigma)));
    result.factors.push_back(gtsam::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
      b_key,
      initial_bias_,
      gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << gtsam::Vector3::Constant(acc_bias_sigma_), gtsam::Vector3::Constant(gyr_bias_sigma_))
          .finished())));

    last_state_ = gtsam::NavState(pose, gtsam::Vector3::Zero());
    last_bias_ = initial_bias_;
    last_key_ = key;
    last_key_time_ = timestamp;
    has_last_key_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] first state %s: velocity prior %.2f m/s (%s)",
      name_.c_str(),
      gtsam::DefaultKeyFormatter(key).c_str(),
      v_sigma,
      still ? "at rest" : "moving");
    return result;
  }

  if (timestamp <= last_key_time_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] state %s at %.3f is not after the previous one (%.3f); no IMU factor",
      name_.c_str(),
      gtsam::DefaultKeyFormatter(key).c_str(),
      timestamp,
      last_key_time_);
    return result;
  }

  gtsam::PreintegratedCombinedMeasurements pim(params_, last_bias_);
  {
    std::lock_guard<std::mutex> lock(imu_mtx_);
    integrate(pim, last_key_time_, timestamp, true);
  }
  if (pim.deltaTij() <= 0.0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] no IMU data between %.3f and %.3f; state %s gets no IMU factor",
      name_.c_str(),
      last_key_time_,
      timestamp,
      gtsam::DefaultKeyFormatter(key).c_str());
    return result;
  }

  result.factors.push_back(gtsam::make_shared<gtsam::CombinedImuFactor>(
    last_key_, velocityKey(last_key_), key, v_key, biasKey(last_key_), b_key, pim));

  const gtsam::NavState predicted = pim.predict(last_state_, last_bias_);
  result.values.insert(key, predicted.pose());
  result.values.insert(v_key, gtsam::Vector3(predicted.velocity()));
  result.values.insert(b_key, last_bias_);

  last_state_ = predicted;  // until the optimizer has the state
  last_key_ = key;
  last_key_time_ = timestamp;
  return result;
}

// ==========================================================================
// onOptimizationComplete
// ==========================================================================

void ImuFactor::onOptimizationComplete(const gtsam::Values & optimized_values, bool /*graph_corrected*/)
{
  if (!has_last_key_) return;
  const gtsam::Key v_key = velocityKey(last_key_);
  const gtsam::Key b_key = biasKey(last_key_);
  if (!optimized_values.exists(last_key_) || !optimized_values.exists(v_key) || !optimized_values.exists(b_key)) {
    return;
  }
  last_state_ = gtsam::NavState(optimized_values.at<gtsam::Pose3>(last_key_), optimized_values.at<gtsam::Vector3>(v_key));
  last_bias_ = optimized_values.at<gtsam::imuBias::ConstantBias>(b_key);
  resetOdometry();
}

// ==========================================================================
// IMU samples
// ==========================================================================

void ImuFactor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!active_) return;

  if (!has_imu_tf_) {
    try {
      const auto tf = tf_->lookupTransform(base_link_frame_, imu_frame_, tf2::TimePointZero);
      const auto & t = tf.transform.translation;
      const auto & r = tf.transform.rotation;
      base_T_imu_ = gtsam::Pose3(gtsam::Rot3::Quaternion(r.w, r.x, r.y, r.z), gtsam::Point3(t.x, t.y, t.z));
      params_->body_P_sensor = base_T_imu_;
      has_imu_tf_ = true;
    } catch (const tf2::TransformException &) {
      return;
    }
  }

  Sample sample;
  sample.t = toSec(msg->header.stamp);
  const Eigen::Matrix3d imu_R_msg = imu_R_msg_.matrix();
  sample.acc =
    imu_R_msg * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  sample.gyr = imu_R_msg * Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  std::lock_guard<std::mutex> lock(imu_mtx_);
  if (!buffer_.empty() && sample.t <= buffer_.back().t) {
    if (sample.t < buffer_.back().t - 1.0) buffer_.clear();  // the clock restarted (bag replayed)
    else return;  // duplicate or out of order
  }
  buffer_.push_back(sample);
  while (buffer_.size() > kMaxImuBuffer) buffer_.pop_front();

  if (!isReady()) {
    // Warmup: consecutive samples at rest
    if (sample.gyr.norm() < stationary_gyr_threshold_) {
      warmup_.push_back(sample);
      if (msg->orientation_covariance[0] >= 0.0) {
        const Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        // world <- imu_frame = (world <- message axes) * (message axes <- imu_frame)
        if (q.squaredNorm() > 0.5) {
          warmup_orientations_.push_back(q.normalized() * Eigen::Quaterniond(imu_R_msg.transpose()));
        }
      }
    } else {
      warmup_.clear();
      warmup_orientations_.clear();
    }
    if (static_cast<int>(warmup_.size()) >= warmup_samples_) finishWarmup();
    return;
  }

  if (!odom_ready_) return;
  const double dt = sample.t - odom_last_time_;
  if (dt <= 0.0) return;
  odom_pim_->integrateMeasurement(sample.acc, sample.gyr, dt);
  odom_last_time_ = sample.t;
  if (state_->load(std::memory_order_acquire) != SlamState::TRACKING) return;
  publishOdometry(odom_pim_->predict(odom_start_, odom_bias_), sample.gyr - odom_bias_.gyroscope(), sample.t);
}

void ImuFactor::finishWarmup()
{
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  for (const auto & s : warmup_) {
    acc += s.acc;
    gyr += s.gyr;
  }
  acc /= static_cast<double>(warmup_.size());
  gyr /= static_cast<double>(warmup_.size());

  // At rest the accelerometer reads gravity: specific force f = R_world_base^T * (0, 0, g).
  const Eigen::Matrix3d base_R_imu = base_T_imu_.rotation().matrix();
  const Eigen::Vector3d f = base_R_imu * acc;
  const bool has_gravity = std::abs(f.norm() - gravity_) < kGravityTolerance * gravity_;
  std::string source;
  if (has_gravity) {
    const double roll = std::atan2(f.y(), f.z());
    const double pitch = std::atan2(-f.x(), std::hypot(f.y(), f.z()));
    gravity_rotation_ = gtsam::Rot3::Ypr(0.0, pitch, roll);
    source = "accelerometer";
  } else if (!warmup_orientations_.empty()) {
    Eigen::Vector4d sum = Eigen::Vector4d::Zero();
    for (const auto & q : warmup_orientations_) {
      const double sign = q.dot(warmup_orientations_.front()) < 0.0 ? -1.0 : 1.0;
      sum += sign * Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
    }
    const Eigen::Quaterniond q_avg = Eigen::Quaterniond(sum(0), sum(1), sum(2), sum(3)).normalized();
    const gtsam::Rot3 world_R_base(q_avg.toRotationMatrix() * base_R_imu.transpose());
    gravity_rotation_ = gtsam::Rot3::Ypr(0.0, world_R_base.pitch(), world_R_base.roll());
    source = "orientation";
  } else {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      10000,
      "[%s] cannot level: %s reads %.2f m/s^2 at rest (gravity %.2f) and has no orientation",
      name_.c_str(),
      imu_topic_.c_str(),
      f.norm(),
      gravity_);
    warmup_.clear();
    return;
  }
  if (!has_gravity && add_factors_) {
    // Preintegration subtracts gravity: a gravity-compensated topic would corrupt the graph.
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] %s reads %.2f m/s^2 at rest, not gravity (%.2f): it is not specific force, so no IMU factors "
      "will be added (use a raw topic such as /novatel/oem7/imu/data_raw)",
      name_.c_str(),
      imu_topic_.c_str(),
      f.norm(),
      gravity_);
    add_factors_ = false;
  }

  // Gyro bias: the mean rate at rest (the earth's rotation, 7e-5 rad/s, is left in it).
  initial_bias_ = gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), gyr);
  odom_start_ = gtsam::NavState(gtsam::Pose3(gravity_rotation_, gtsam::Point3::Zero()), gtsam::Vector3::Zero());
  odom_bias_ = initial_bias_;
  odom_pim_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(params_, odom_bias_);
  odom_last_time_ = warmup_.back().t;
  odom_ready_ = true;
  warmup_.clear();
  warmup_orientations_.clear();
  warmup_complete_.store(true, std::memory_order_release);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] IMU warmup complete from the %s: roll %.2f deg, pitch %.2f deg, |f| %.3f m/s^2, gyro bias "
    "(%.1e, %.1e, %.1e) rad/s",
    name_.c_str(),
    source.c_str(),
    gravity_rotation_.roll() * 180.0 / M_PI,
    gravity_rotation_.pitch() * 180.0 / M_PI,
    f.norm(),
    gyr.x(),
    gyr.y(),
    gyr.z());
}

bool ImuFactor::stationaryBefore(double t, double window) const
{
  bool any = false;
  for (const auto & s : buffer_) {
    if (s.t < t - window) continue;
    if (s.t > t) break;
    any = true;
    if ((s.gyr - initial_bias_.gyroscope()).norm() >= stationary_gyr_threshold_) return false;
  }
  return any;
}

void ImuFactor::integrate(gtsam::PreintegratedCombinedMeasurements & pim, double from, double to, bool drain)
{
  // Each sample holds over the interval that ends at its stamp.
  double prev = from;
  bool gap_warned = false;
  for (const auto & s : buffer_) {
    if (s.t <= from) continue;
    const double end = std::min(s.t, to);
    const double dt = end - prev;
    if (dt > 0.0) {
      if (dt > kMaxImuDt && drain && !gap_warned) {  // once per gap: the graph integrates each span once
        RCLCPP_WARN(node_->get_logger(), "[%s] %.3f s gap in the IMU data at %.3f", name_.c_str(), dt, prev);
        gap_warned = true;
      }
      pim.integrateMeasurement(s.acc, s.gyr, dt);
      prev = end;
    }
    if (s.t >= to) break;
  }
  if (prev < to && !buffer_.empty() && buffer_.back().t > from) {
    // No sample after `to` yet: hold the newest one to the end.
    pim.integrateMeasurement(buffer_.back().acc, buffer_.back().gyr, to - prev);
  }
  if (drain) {
    while (!buffer_.empty() && buffer_.front().t <= to) buffer_.pop_front();
  }
}

void ImuFactor::resetOdometry()
{
  std::lock_guard<std::mutex> lock(imu_mtx_);
  odom_start_ = last_state_;
  odom_bias_ = last_bias_;
  odom_pim_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(params_, odom_bias_);
  odom_last_time_ = last_key_time_;
  if (!buffer_.empty() && buffer_.back().t > last_key_time_) {
    integrate(*odom_pim_, last_key_time_, buffer_.back().t, false);
    odom_last_time_ = buffer_.back().t;
  }
  odom_ready_ = true;
}

void ImuFactor::publishOdometry(const gtsam::NavState & state, const Eigen::Vector3d & gyr_imu, double t)
{
  const gtsam::Pose3 pose = state.pose();
  setOdomPose(pose);
  if (!odom_pub_->is_activated()) return;

  const Eigen::Vector3d v_body = pose.rotation().matrix().transpose() * state.velocity();
  const Eigen::Vector3d w_body = base_T_imu_.rotation().matrix() * gyr_imu;
  const auto q = pose.rotation().toQuaternion();

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
  msg.header.frame_id = odom_frame_;
  msg.child_frame_id = base_link_frame_;
  msg.pose.pose.position.x = pose.x();
  msg.pose.pose.position.y = pose.y();
  msg.pose.pose.position.z = pose.z();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
  msg.twist.twist.linear.x = v_body.x();
  msg.twist.twist.linear.y = v_body.y();
  msg.twist.twist.linear.z = v_body.z();
  msg.twist.twist.angular.x = w_body.x();
  msg.twist.twist.angular.y = w_body.y();
  msg.twist.twist.angular.z = w_body.z();
  odom_pub_->publish(msg);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::ImuFactor, eidos::FactorPlugin)
