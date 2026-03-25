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
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <tf2_ros/buffer.h>

#include <pluginlib/class_list_macros.hpp>

namespace eidos
{

static constexpr double kMaxImuDt = 0.1;
static constexpr double kBiasNoisePerSec = 1e-3;

// ==========================================================================
// Lifecycle
// ==========================================================================

void ImuFactor::onInitialize()
{
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".imu_topic", std::string("/imu/data"));
  node_->declare_parameter(prefix + ".imu_frame", std::string("imu_link"));
  node_->declare_parameter(prefix + ".acc_cov", std::vector<double>{9e-6, 9e-6, 9e-6});
  node_->declare_parameter(prefix + ".gyr_cov", std::vector<double>{1e-6, 1e-6, 1e-6});
  node_->declare_parameter(prefix + ".integration_cov", std::vector<double>{1e-4, 1e-4, 1e-4});
  node_->declare_parameter(prefix + ".gravity", gravity_);
  node_->declare_parameter(prefix + ".default_imu_dt", default_imu_dt_);

  node_->get_parameter(prefix + ".imu_topic", imu_topic_);
  node_->get_parameter(prefix + ".imu_frame", imu_frame_);
  node_->get_parameter(prefix + ".acc_cov", acc_cov_);
  node_->get_parameter(prefix + ".gyr_cov", gyr_cov_);
  node_->get_parameter(prefix + ".integration_cov", integration_cov_);
  node_->get_parameter(prefix + ".gravity", gravity_);
  node_->get_parameter(prefix + ".default_imu_dt", default_imu_dt_);
  node_->get_parameter("frames.odometry", odom_frame_);
  node_->get_parameter("frames.base_link", base_link_frame_);

  // Set up GTSAM preintegration params
  auto p = gtsam::PreintegrationParams::MakeSharedU(gravity_);  // returns boost::shared_ptr
  if (acc_cov_.size() >= 3) {
    p->accelerometerCovariance =
      Eigen::Vector3d(acc_cov_[0], acc_cov_[1], acc_cov_[2]).asDiagonal();
  }
  if (gyr_cov_.size() >= 3) {
    p->gyroscopeCovariance = Eigen::Vector3d(gyr_cov_[0], gyr_cov_[1], gyr_cov_[2]).asDiagonal();
  }
  if (integration_cov_.size() >= 3) {
    p->integrationCovariance =
      Eigen::Vector3d(integration_cov_[0], integration_cov_[1], integration_cov_[2]).asDiagonal();
  }
  preint_params_ = p;
  integrator_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(p, current_bias_);

  // IMU subscription
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ImuFactor::imuCallback, this, std::placeholders::_1), sub_opts);

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(name_ + "/odometry", 10);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (IMU preintegration factor)", name_.c_str());
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
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

// ==========================================================================
// latchFactor — attach ImuFactor between previous and current state
// ==========================================================================

StampedFactorResult ImuFactor::latchFactor(gtsam::Key key, double timestamp)
{
  StampedFactorResult result;
  if (!active_ || !has_last_key_) {
    // First state — just record, can't produce a between-factor yet
    last_key_ = key;
    last_key_time_ = timestamp;
    has_last_key_ = true;
    // Reset integrator for next segment
    integrator_->resetIntegrationAndSetBias(current_bias_);
    return result;
  }

  // Drain IMU buffer for measurements between last_key_time_ and timestamp
  {
    std::lock_guard lock(imu_mtx_);
    while (!imu_buffer_.empty()) {
      double t = rclcpp::Time(imu_buffer_.front().header.stamp).seconds();
      if (t > timestamp) break;
      if (t > last_key_time_) {
        auto converted = convertToBaseFrame(imu_buffer_.front());
        double dt =
          (last_imu_time_ > 0.0) ? (t - last_imu_time_) : default_imu_dt_;
        if (dt > 0.0 && dt < kMaxImuDt) {
          Eigen::Vector3d acc(
            converted.linear_acceleration.x, converted.linear_acceleration.y,
            converted.linear_acceleration.z);
          Eigen::Vector3d gyr(
            converted.angular_velocity.x, converted.angular_velocity.y,
            converted.angular_velocity.z);
          integrator_->integrateMeasurement(acc, gyr, dt);
        }
        last_imu_time_ = t;
      }
      imu_buffer_.pop_front();
    }
  }

  // Produce a BetweenFactor<Pose3> from the preintegrated pose delta.
  // Full ImuFactor with velocity + bias states requires velocity/bias keys
  // in the graph — for now we extract just the pose component.
  gtsam::Symbol prev_x(last_key_);
  gtsam::Symbol curr_x(key);

  gtsam::NavState predicted = integrator_->predict(reference_state_, current_bias_);
  gtsam::Pose3 preint_delta = reference_state_.pose().between(predicted.pose());

  auto noise = gtsam::noiseModel::Gaussian::Covariance(
    integrator_->preintMeasCov().block<6, 6>(0, 0));
  result.factors.push_back(
    gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(last_key_, key, preint_delta, noise));

  RCLCPP_INFO(
    node_->get_logger(),
    "\033[34m[%s] ImuBetween (%c,%lu)->(%c,%lu) dt=%.3f\033[0m",
    name_.c_str(), prev_x.chr(), prev_x.index(), curr_x.chr(), curr_x.index(),
    timestamp - last_key_time_);

  // Update tracking
  last_key_ = key;
  last_key_time_ = timestamp;
  integrator_->resetIntegrationAndSetBias(current_bias_);

  return result;
}

// ==========================================================================
// onOptimizationComplete
// ==========================================================================

void ImuFactor::onOptimizationComplete(
  const gtsam::Values & optimized_values, bool /*loop_closure_detected*/)
{
  // Update reference state from latest optimized pose
  if (has_last_key_ && optimized_values.exists(last_key_)) {
    auto corrected = optimized_values.at<gtsam::Pose3>(last_key_);
    reference_state_ = gtsam::NavState(corrected, gtsam::Vector3::Zero());
  }
}

// ==========================================================================
// IMU callback
// ==========================================================================

void ImuFactor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!active_) return;

  // Resolve base_link ← imu TF (once)
  if (!has_imu_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, imu_frame_, tf2::TimePointZero);
      const auto & t = tf_msg.transform.translation;
      const auto & r = tf_msg.transform.rotation;
      R_base_imu_ = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      t_base_imu_ = Eigen::Vector3d(t.x, t.y, t.z);
      has_imu_tf_ = true;
    } catch (const tf2::TransformException &) {
      return;
    }
  }

  // Buffer for latchFactor consumption
  {
    std::lock_guard lock(imu_mtx_);
    imu_buffer_.push_back(*msg);
    while (imu_buffer_.size() > kMaxImuBuffer) imu_buffer_.pop_front();
  }

  // Live preintegration for odom output
  if (state_->load(std::memory_order_acquire) != SlamState::TRACKING) return;

  auto converted = convertToBaseFrame(*msg);
  double t = rclcpp::Time(msg->header.stamp).seconds();
  double dt = (last_imu_time_ > 0.0) ? (t - last_imu_time_) : default_imu_dt_;

  if (dt > 0.0 && dt < kMaxImuDt) {
    Eigen::Vector3d acc(
      converted.linear_acceleration.x, converted.linear_acceleration.y,
      converted.linear_acceleration.z);
    Eigen::Vector3d gyr(
      converted.angular_velocity.x, converted.angular_velocity.y, converted.angular_velocity.z);

    // Live prediction for odom output
    auto state = integrator_->predict(reference_state_, current_bias_);
    gtsam::Pose3 base_pose(state.pose().rotation(), state.pose().translation());
    setOdomPose(base_pose);
  }
  last_imu_time_ = t;
}

// ==========================================================================
// Frame conversion
// ==========================================================================

sensor_msgs::msg::Imu ImuFactor::convertToBaseFrame(const sensor_msgs::msg::Imu & imu_msg)
{
  sensor_msgs::msg::Imu converted = imu_msg;

  Eigen::Vector3d acc(
    imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
  Eigen::Vector3d gyr(
    imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

  Eigen::Vector3d acc_base = R_base_imu_ * acc;
  Eigen::Vector3d gyr_base = R_base_imu_ * gyr;

  converted.linear_acceleration.x = acc_base.x();
  converted.linear_acceleration.y = acc_base.y();
  converted.linear_acceleration.z = acc_base.z();
  converted.angular_velocity.x = gyr_base.x();
  converted.angular_velocity.y = gyr_base.y();
  converted.angular_velocity.z = gyr_base.z();

  return converted;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::ImuFactor, eidos::FactorPlugin)
