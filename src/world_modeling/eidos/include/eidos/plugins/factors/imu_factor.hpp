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

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegratedRotation.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"

namespace eidos
{

/**
 * @brief IMU preintegration factor plugin.
 *
 * Subscribes to IMU, performs GTSAM preintegration, and produces ImuFactor +
 * BetweenFactor<imuBias::ConstantBias> constraints between consecutive states.
 *
 * Can create its own states (produceFactor) or latch onto states created by
 * other plugins (latchFactor).
 *
 * Also writes an odom-frame pose via setOdomPose() for eidos_transform
 * measurement correction — this is how IMU data reaches the TF chain.
 */
class ImuFactor : public FactorPlugin
{
public:
  ImuFactor() = default;
  ~ImuFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  StampedFactorResult latchFactor(gtsam::Key key, double timestamp) override;

  bool isReady() const override
  {
    return warmup_complete_;
  }

  void onOptimizationComplete(const gtsam::Values & optimized_values, bool loop_closure_detected) override;

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /// Convert IMU message from IMU frame to base_link frame.
  sensor_msgs::msg::Imu convertToBaseFrame(const sensor_msgs::msg::Imu & imu_msg);

  // ---- Subscriptions + publishers ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // ---- Preintegration ----
  boost::shared_ptr<gtsam::PreintegrationParams> preint_params_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> integrator_;
  gtsam::NavState reference_state_;
  gtsam::imuBias::ConstantBias reference_bias_;
  gtsam::imuBias::ConstantBias current_bias_;

  // ---- Tracking ----
  gtsam::Key last_key_{0};
  double last_key_time_ = 0.0;
  bool has_last_key_ = false;

  // ---- IMU buffer for latching ----
  std::deque<sensor_msgs::msg::Imu> imu_buffer_;
  std::mutex imu_mtx_;
  double last_imu_time_ = 0.0;

  // ---- Extrinsics ----
  Eigen::Matrix3d R_base_imu_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_base_imu_ = Eigen::Vector3d::Zero();
  bool has_imu_tf_ = false;

  // ---- Warmup ----
  bool warmup_complete_ = false;
  int warmup_count_ = 0;
  int warmup_samples_ = 200;
  double stationary_gyr_threshold_ = 0.005;
  Eigen::Vector4d warmup_quat_sum_{0, 0, 0, 0};
  bool warmup_quat_hemisphere_set_ = false;
  Eigen::Quaterniond warmup_quat_reference_;
  gtsam::Rot3 initial_gravity_orientation_;

  // ---- Parameters ----
  std::string imu_topic_;
  std::string imu_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;
  std::vector<double> acc_cov_;
  std::vector<double> gyr_cov_;
  std::vector<double> integration_cov_;
  double gravity_ = 9.80511;
  double default_imu_dt_ = 0.002;
  bool active_ = false;
  bool add_factors_ = true;

  static constexpr size_t kMaxImuBuffer = 5000;
};

}  // namespace eidos
