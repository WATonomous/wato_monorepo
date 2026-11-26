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

#include "eidos/plugins/factors/gravity_factor.hpp"

#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <pluginlib/class_list_macros.hpp>

namespace eidos
{

constexpr double kBufferSeconds = 20.0;  ///< History kept for windows ending at a delayed state
constexpr double kMaxSampleGap = 0.1;  ///< Largest IMU gap allowed inside a window (s)
constexpr double kQuatSquaredNormMin = 0.5;
constexpr size_t kSummaryEvery = 100;  ///< Log a summary every this many attempts

// ==========================================================================
// Lifecycle
// ==========================================================================

void GravityFactor::onInitialize()
{
  const std::string & p = name_;
  node_->declare_parameter(p + ".add_factors", add_factors_);
  node_->declare_parameter(p + ".mode", mode_);
  node_->declare_parameter(p + ".imu_topic", imu_topic_);
  node_->declare_parameter(p + ".imu_frame", imu_frame_);
  node_->declare_parameter(p + ".velocity_topic", velocity_topic_);
  node_->declare_parameter(p + ".window", window_);
  node_->declare_parameter(p + ".sigma", sigma_);
  node_->declare_parameter(p + ".huber_k", huber_k_);
  node_->declare_parameter(p + ".gravity", gravity_);
  node_->declare_parameter(p + ".max_accel_deviation", max_accel_deviation_);
  node_->declare_parameter(p + ".max_angular_rate", max_angular_rate_);
  node_->declare_parameter(p + ".min_imu_samples", min_imu_samples_);

  node_->get_parameter(p + ".add_factors", add_factors_);
  node_->get_parameter(p + ".mode", mode_);
  node_->get_parameter(p + ".imu_topic", imu_topic_);
  node_->get_parameter(p + ".imu_frame", imu_frame_);
  node_->get_parameter(p + ".velocity_topic", velocity_topic_);
  node_->get_parameter(p + ".window", window_);
  node_->get_parameter(p + ".sigma", sigma_);
  node_->get_parameter(p + ".huber_k", huber_k_);
  node_->get_parameter(p + ".gravity", gravity_);
  node_->get_parameter(p + ".max_accel_deviation", max_accel_deviation_);
  node_->get_parameter(p + ".max_angular_rate", max_angular_rate_);
  node_->get_parameter(p + ".min_imu_samples", min_imu_samples_);
  node_->get_parameter("frames.base_link", base_link_frame_);

  if (mode_ != "accelerometer" && mode_ != "orientation") {
    RCLCPP_WARN(node_->get_logger(), "[%s] unknown mode '%s', using 'accelerometer'", name_.c_str(), mode_.c_str());
    mode_ = "accelerometer";
  }

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort(),
    std::bind(&GravityFactor::imuCallback, this, std::placeholders::_1),
    sub_opts);
  if (mode_ == "accelerometer" && !velocity_topic_.empty()) {
    velocity_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      velocity_topic_,
      rclcpp::QoS(rclcpp::KeepLast(100)),
      std::bind(&GravityFactor::velocityCallback, this, std::placeholders::_1),
      sub_opts);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] initialized (roll/pitch prior, mode=%s, velocity=%s, window=%.2f s, sigma=%.3f rad, huber=%.2f)",
    name_.c_str(),
    mode_.c_str(),
    velocity_sub_ ? velocity_topic_.c_str() : "none",
    window_,
    sigma_,
    huber_k_);
}

void GravityFactor::activate()
{
  active_ = true;
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void GravityFactor::deactivate()
{
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

// ==========================================================================
// latchFactor
// ==========================================================================

StampedFactorResult GravityFactor::latchFactor(gtsam::Key key, double timestamp)
{
  StampedFactorResult result;
  if (!active_ || !add_factors_) return result;

  const double t0 = timestamp - window_;
  Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero(), gyro_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d up_sum = Eigen::Vector3d::Zero();
  int n = 0, n_up = 0;
  double first_t = 0.0, last_t = 0.0, prev_t = 0.0;
  bool gap = false;
  {
    std::lock_guard lock(buffer_mtx_);
    for (const auto & s : imu_buffer_) {
      if (s.t < t0) continue;
      if (s.t > timestamp) break;
      if (n == 0) first_t = s.t;
      if (n > 0 && s.t - prev_t > kMaxSampleGap) gap = true;
      prev_t = last_t = s.t;
      accel_sum += s.accel;
      gyro_sum += s.gyro;
      if (s.up) {
        up_sum += *s.up;
        n_up++;
      }
      n++;
    }
  }

  // The window must be covered: enough samples, no gap, reaching both ends
  const bool covered =
    n >= min_imu_samples_ && !gap && first_t - t0 < kMaxSampleGap && timestamp - last_t < kMaxSampleGap;

  std::optional<Eigen::Vector3d> up_body;
  bool dynamics_ok = true;
  if (covered) {
    const Eigen::Vector3d gyro = gyro_sum / n;
    if (gyro.norm() > max_angular_rate_) {
      dynamics_ok = false;
    } else if (mode_ == "orientation") {
      if (n_up == n && up_sum.norm() > 0.0) up_body = up_sum.normalized();
    } else {
      // Specific force f = a - g_body, so "up" (scaled by g) is f - a
      Eigen::Vector3d up = accel_sum / n;
      if (velocity_sub_) {
        auto a = bodyAcceleration(t0, timestamp, gyro);
        if (a) {
          up -= *a;
        } else {
          up = Eigen::Vector3d::Zero();  // velocity does not cover the window
        }
      }
      if (up.norm() > 0.0) {
        if (std::abs(up.norm() - gravity_) > max_accel_deviation_) {
          dynamics_ok = false;
        } else {
          up_body = up.normalized();
        }
      }
    }
  }

  if (!up_body) {
    (dynamics_ok ? skipped_data_ : skipped_dynamics_)++;
  } else {
    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Isotropic::Sigma(2, sigma_);
    if (huber_k_ > 0.0) {
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(huber_k_), noise);
    }
    // Rotating the measured body-frame "up" into the map frame must give the map's "up"
    result.factors.push_back(gtsam::make_shared<gtsam::Pose3AttitudeFactor>(
      key, gtsam::Unit3(0.0, 0.0, 1.0), noise, gtsam::Unit3(up_body->x(), up_body->y(), up_body->z())));
    added_++;
  }

  const size_t attempts = added_ + skipped_dynamics_ + skipped_data_;
  if (attempts % kSummaryEvery == 1) {
    const auto & u = up_body.value_or(Eigen::Vector3d::Zero());
    gtsam::Symbol sym(key);
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] (%c,%lu): %s | measured up in body=(%.3f,%.3f,%.3f), tilt %.2f deg | "
      "so far %zu added, %zu skipped (dynamics), %zu skipped (no data)",
      name_.c_str(),
      sym.chr(),
      sym.index(),
      up_body ? "factor added" : (dynamics_ok ? "no data" : "skipped, dynamics"),
      u.x(),
      u.y(),
      u.z(),
      std::acos(std::clamp(u.z(), -1.0, 1.0)) * 180.0 / M_PI,
      added_,
      skipped_dynamics_,
      skipped_data_);
  }
  return result;
}

// ==========================================================================
// Vehicle acceleration from the velocity topic
// ==========================================================================

std::optional<Eigen::Vector3d> GravityFactor::bodyAcceleration(double t0, double t1, const Eigen::Vector3d & gyro)
{
  // Mean velocity over the first and last third of the window; their difference over the time
  // between their mean stamps is dv/dt, and w x v is the rotation of the body frame
  const double third = (t1 - t0) / 3.0;
  Eigen::Vector3d va = Eigen::Vector3d::Zero(), vb = Eigen::Vector3d::Zero(), vm = Eigen::Vector3d::Zero();
  double ta = 0.0, tb = 0.0;
  int na = 0, nb = 0, nm = 0;
  {
    std::lock_guard lock(buffer_mtx_);
    for (const auto & s : velocity_buffer_) {
      if (s.t < t0) continue;
      if (s.t > t1) break;
      if (s.t <= t0 + third) {
        va += s.v;
        ta += s.t;
        na++;
      } else if (s.t >= t1 - third) {
        vb += s.v;
        tb += s.t;
        nb++;
      }
      vm += s.v;
      nm++;
    }
  }
  if (na == 0 || nb == 0) return std::nullopt;
  va /= na;
  vb /= nb;
  vm /= nm;
  const double dt = tb / nb - ta / na;
  if (dt <= 0.0) return std::nullopt;
  return Eigen::Vector3d((vb - va) / dt + gyro.cross(vm));
}

// ==========================================================================
// Callbacks
// ==========================================================================

void GravityFactor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!has_imu_tf_) {
    const std::string frame = imu_frame_.empty() ? msg->header.frame_id : imu_frame_;
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, frame, tf2::TimePointZero);
      const auto & r = tf_msg.transform.rotation;
      R_base_imu_ = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      has_imu_tf_ = true;
    } catch (const tf2::TransformException &) {
      return;
    }
  }

  ImuSample s;
  s.t = rclcpp::Time(msg->header.stamp).seconds();
  const auto & a = msg->linear_acceleration;
  const auto & w = msg->angular_velocity;
  s.accel = R_base_imu_ * Eigen::Vector3d(a.x, a.y, a.z);
  s.gyro = R_base_imu_ * Eigen::Vector3d(w.x, w.y, w.z);
  const auto & q = msg->orientation;
  Eigen::Quaterniond q_world_imu(q.w, q.x, q.y, q.z);
  if (msg->orientation_covariance[0] >= 0.0 && q_world_imu.squaredNorm() > kQuatSquaredNormMin) {
    // World "up" expressed in the IMU frame, then in base_link
    s.up = Eigen::Vector3d(R_base_imu_ * (q_world_imu.normalized().conjugate() * Eigen::Vector3d::UnitZ()));
  }

  std::lock_guard lock(buffer_mtx_);
  imu_buffer_.push_back(s);
  while (!imu_buffer_.empty() && imu_buffer_.front().t < s.t - kBufferSeconds) imu_buffer_.pop_front();
}

void GravityFactor::velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  VelocitySample s;
  s.t = rclcpp::Time(msg->header.stamp).seconds();
  const auto & v = msg->twist.twist.linear;
  s.v = Eigen::Vector3d(v.x, v.y, v.z);
  // An odometry message without twist (first match, or a long gap) carries zeros; skip it
  if (s.v.isZero()) return;

  std::lock_guard lock(buffer_mtx_);
  velocity_buffer_.push_back(s);
  while (!velocity_buffer_.empty() && velocity_buffer_.front().t < s.t - kBufferSeconds) velocity_buffer_.pop_front();
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GravityFactor, eidos::FactorPlugin)
