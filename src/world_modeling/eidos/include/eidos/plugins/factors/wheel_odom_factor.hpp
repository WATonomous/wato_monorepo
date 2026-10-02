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

#pragma once

#include <deque>
#include <Eigen/Core>
#include <mutex>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/utils/wheel_kinematics.hpp"

namespace eidos
{

/**
 * @brief Wheel odometry factor plugin.
 *
 * Combines rear-axle wheel speed (from sensor_msgs/JointState wheel joint velocities)
 * with IMU yaw rate. The axle velocity is transferred to the base frame using the
 * axle lever arm from TF. Publishes nav_msgs/Odometry with a per-reading twist
 * covariance (wide when the wheels read zero), for eidos_transform to fuse.
 *
 * When add_factors is true, latchFactor() attaches a planar BetweenFactor<Pose3>
 * between consecutive states created by other plugins. The plugin only consumes
 * sensor topics; it never reads other plugins' outputs.
 */
class WheelOdomFactor : public FactorPlugin
{
public:
  WheelOdomFactor() = default;
  ~WheelOdomFactor() override = default;

  void onInitialize() override;

  void activate() override;

  void deactivate() override;

  StampedFactorResult latchFactor(gtsam::Key key, double timestamp) override;

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /// Resolve the IMU and axle extrinsics from TF. Returns true once both are cached.
  bool resolveExtrinsics();

  void publishOdometry(const builtin_interfaces::msg::Time & stamp, const wheel_kinematics::Sample & sample);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Latest IMU yaw rate in base frame (guarded by imu_mtx_)
  std::mutex imu_mtx_;
  Eigen::Vector3d latest_gyro_base_ = Eigen::Vector3d::Zero();
  bool has_gyro_ = false;

  // Extrinsics (resolved once from TF)
  Eigen::Matrix3d R_base_imu_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_base_axle_ = Eigen::Vector3d::Zero();
  bool has_imu_tf_ = false;
  bool has_axle_tf_ = false;

  // Samples for latchFactor (guarded by sample_mtx_)
  std::mutex sample_mtx_;
  std::deque<wheel_kinematics::Sample> samples_;

  // Live odometry state (wheel callback only)
  wheel_kinematics::Pose2 odom_pose_;
  double last_sample_time_ = 0.0;

  // latchFactor bookkeeping
  gtsam::Key last_key_{0};
  double last_key_time_ = 0.0;
  bool has_last_key_ = false;

  // Parameters
  std::string joint_states_topic_;
  std::string left_joint_;
  std::string right_joint_;
  double wheel_radius_ = 0.31235;
  std::string axle_frame_;
  std::string imu_topic_;
  std::string imu_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;
  wheel_kinematics::SpeedNoise speed_noise_;
  wheel_kinematics::FactorNoise factor_noise_;
  double max_dt_ = 0.5;
  bool active_ = false;
  bool add_factors_ = false;

  static constexpr size_t kMaxSampleBuffer = 2000;
};

}  // namespace eidos
