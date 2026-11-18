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

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <atomic>
#include <deque>
#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <optional>
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
 * Latches onto the states other plugins create. Every state i gets a velocity V(i) and an IMU bias
 * B(i) of its own (gtsam::Symbol('v', i), gtsam::Symbol('b', i)), and consecutive states are joined
 * by a gtsam::CombinedImuFactor: the IMU measurements between them, preintegrated, constrain the
 * relative pose, the velocities and the bias random walk. The first state gets priors on its
 * velocity (tight if the IMU shows the vehicle stationary) and bias (from the stationary warmup).
 *
 * It also offers the IMU's prediction of each new state's pose as its initial value, which eidos
 * uses when the state's creator gives none.
 *
 * Measurements are integrated in the IMU frame with body_P_sensor set to the base_link <- IMU
 * extrinsic from TF, so the accelerometer must report specific force including gravity (e.g.
 * /novatel/oem7/imu/data_raw, not a gravity-compensated topic).
 *
 * With add_factors false it adds nothing to the graph and only publishes odometry integrated from
 * the warmup orientation, for eidos_transform.
 */
class ImuFactor : public FactorPlugin
{
public:
  ImuFactor() = default;
  ~ImuFactor() override = default;

  /// @brief Declare ROS parameters, create IMU subscription and odometry publisher.
  void onInitialize() override;

  /// @brief Enable IMU subscription and begin preintegration.
  void activate() override;

  /// @brief Disable IMU subscription and stop processing.
  void deactivate() override;

  /**
   * @brief Attach the preintegrated IMU factor between the previous state and this one.
   *
   * The first state gets its velocity and bias priors instead. Both cases also return initial
   * values for the new velocity and bias, and the IMU's prediction of the new state's pose.
   *
   * @param key GTSAM key of the newly created state.
   * @param timestamp Timestamp of the new state (seconds).
   * @return StampedFactorResult with the factors and initial values, or empty if add_factors is false.
   */
  StampedFactorResult latchFactor(gtsam::Key key, double timestamp) override;

  /**
   * @brief Whether IMU warmup (stationarity detection) is complete.
   * @return true once enough stationary IMU samples have been collected.
   */
  bool isReady() const override
  {
    return warmup_complete_.load(std::memory_order_acquire);
  }

  /// @brief Remember the pose tracking starts from (its heading and position seed the first state).
  void onTrackingBegin(const gtsam::Pose3 & pose) override;

  /**
   * @brief Take the optimized pose, velocity and bias of the newest state as the integration start.
   * @param optimized_values Full optimized GTSAM values after ISAM2 update.
   * @param graph_corrected Whether a significant correction (loop closure or GPS) occurred.
   */
  void onOptimizationComplete(const gtsam::Values & optimized_values, bool graph_corrected) override;

  /// @brief Velocity key of state key (same index, character 'v').
  static gtsam::Key velocityKey(gtsam::Key state);

  /// @brief Bias key of state key (same index, character 'b').
  static gtsam::Key biasKey(gtsam::Key state);

private:
  struct Sample
  {
    double t;
    Eigen::Vector3d acc;  ///< specific force, IMU frame (m/s^2)
    Eigen::Vector3d gyr;  ///< angular rate, IMU frame (rad/s)
  };

  using Params = gtsam::PreintegrationCombinedParams;
  using ParamsPtr = decltype(Params::MakeSharedU());

  /// @brief Buffer a sample, run the warmup, and integrate live odometry.
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /// @brief Gravity-aligned orientation, gyro bias and stationarity from the warmup samples.
  void finishWarmup();

  /// @brief Whether the IMU saw no rotation in the window before t (gyro below the threshold).
  bool stationaryBefore(double t, double window) const;

  /// @brief Integrate the buffered samples in (from, to] into pim; drops samples up to `to` if drain.
  void integrate(gtsam::PreintegratedCombinedMeasurements & pim, double from, double to, bool drain);

  /// @brief Re-integrate the live odometry from the newest graph state over the buffered samples.
  void resetOdometry();

  /// @brief Publish the live odometry (pose, body velocity, angular rate in base_link).
  void publishOdometry(const gtsam::NavState & state, const Eigen::Vector3d & gyr_imu, double t);

  // ---- Subscriptions + publishers ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // ---- Preintegration ----
  ParamsPtr params_;
  gtsam::imuBias::ConstantBias initial_bias_;  ///< from the warmup: gyro mean at rest, zero accel

  // ---- Graph state (SLAM thread) ----
  bool has_last_key_ = false;
  gtsam::Key last_key_{0};
  double last_key_time_ = 0.0;
  gtsam::NavState last_state_;  ///< newest state: optimized, or predicted until it is
  gtsam::imuBias::ConstantBias last_bias_;
  gtsam::Pose3 tracking_start_;

  // ---- IMU samples and live odometry (guarded by imu_mtx_) ----
  mutable std::mutex imu_mtx_;
  std::deque<Sample> buffer_;
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> odom_pim_;
  gtsam::NavState odom_start_;
  gtsam::imuBias::ConstantBias odom_bias_;
  double odom_last_time_ = 0.0;
  bool odom_ready_ = false;

  // ---- Extrinsics ----
  gtsam::Rot3 imu_R_msg_;  ///< from the message's axes to imu_frame (axes_rotation_rpy)
  gtsam::Pose3 base_T_imu_;
  bool has_imu_tf_ = false;

  // ---- Warmup ----
  std::atomic<bool> warmup_complete_{false};
  std::vector<Sample> warmup_;
  std::vector<Eigen::Quaterniond> warmup_orientations_;
  gtsam::Rot3 gravity_rotation_;  ///< roll and pitch of base_link at rest, yaw 0

  // ---- Parameters ----
  std::string imu_topic_;
  std::string imu_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;
  std::vector<double> acc_cov_;
  std::vector<double> gyr_cov_;
  std::vector<double> integration_cov_;
  std::vector<double> acc_bias_cov_;
  std::vector<double> gyr_bias_cov_;
  std::vector<double> axes_rotation_rpy_;
  double gravity_ = 9.80511;
  int warmup_samples_ = 200;
  double stationary_gyr_threshold_ = 0.005;
  double velocity_sigma_stationary_ = 0.05;
  double velocity_sigma_moving_ = 10.0;
  double acc_bias_sigma_ = 0.1;
  double gyr_bias_sigma_ = 0.002;
  std::atomic<bool> active_{false};
  std::atomic<bool> add_factors_{true};

  static constexpr size_t kMaxImuBuffer = 5000;
  static constexpr double kMaxImuDt = 0.1;
};

}  // namespace eidos
