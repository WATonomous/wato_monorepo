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

#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>

#include <deque>
#include <Eigen/Geometry>
#include <mutex>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "eidos/plugins/base_motion_model_plugin.hpp"

namespace eidos
{

/**
 * @brief IMU-based high-rate odom filler.
 *
 * Subscribes to IMU data, performs stationarity detection for warmup,
 * then integrates IMU measurements to produce a smooth odom-frame pose
 * at IMU rate (~500 Hz).
 *
 * Does NOT contribute factors to the SLAM graph. TransformManager reads
 * the odom pose via getOdomPose() (lock-free) for odom→base TF.
 *
 * Threading:
 * - IMU callback: own callback group. Calls setOdomPose() (lock-free write).
 * - TransformManager: reads getOdomPose() on its own timer (lock-free read).
 * - No mutexes on the hot path.
 */
class ImuMotionModel : public MotionModelPlugin
{
public:
  ImuMotionModel() = default;
  ~ImuMotionModel() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  bool isReady() const override;
  std::string getReadyStatus() const override;

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // ---- Sub-methods of imuCallback ----
  void updateStationaryDetection(const sensor_msgs::msg::Imu & imu_converted);
  gtsam::NavState integrateSingleAndPredict(const sensor_msgs::msg::Imu & imu_converted);
  void publishIncrementalOdom(
    const sensor_msgs::msg::Imu & imu_converted, const gtsam::Pose3 & base_pose, const gtsam::NavState & state);

  // ---- IMU extrinsic conversion ----
  sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu & imu_in);
  static std::pair<gtsam::Vector3, gtsam::Vector3> extractMeasurement(const sensor_msgs::msg::Imu & msg);
  static bool isValidImuMessage(const sensor_msgs::msg::Imu & msg);

  // ---- Subscription + publisher ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_pub_;

  // ---- Stationarity detection (warmup) ----
  std::deque<sensor_msgs::msg::Imu> stationary_buffer_;
  std::atomic<double> stationary_acc_rms_{0.0};
  std::atomic<double> stationary_gyr_rms_{0.0};
  std::atomic<int> stationary_count_{0};

  // ---- GTSAM preintegration (for real-time odom prediction only) ----
  boost::shared_ptr<gtsam::PreintegrationParams> preint_params_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_integrator_;
  gtsam::NavState odom_reference_state_;  ///< Anchor for forward prediction
  gtsam::imuBias::ConstantBias odom_reference_bias_;

  double last_imu_t_ = -1;
  bool active_ = false;

  // ---- Parameters ----
  std::vector<double> acc_cov_;
  std::vector<double> gyr_cov_;
  std::vector<double> integration_cov_;
  double gravity_;
  double default_imu_dt_;
  double quaternion_norm_threshold_;
  double stationary_acc_threshold_;
  double stationary_gyr_threshold_;
  int stationary_samples_;

  std::vector<double> odom_pose_cov_;
  std::vector<double> odom_twist_cov_;

  // ---- Cached static TF ----
  geometry_msgs::msg::TransformStamped tf_imu_to_base_msg_;
  bool extrinsics_resolved_ = false;

  // ---- Frame names ----
  std::string odom_frame_ = "odom";
  std::string base_link_frame_ = "base_link";
  std::string imu_frame_ = "imu_link";
};

}  // namespace eidos
