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

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include <Eigen/Dense>

#include "eidos_transform/ekf_model_plugin.hpp"

namespace eidos_transform
{

/**
 * @brief 15-state holonomic EKF: pose (6) + velocity (6) + accel bias (3).
 *
 * State layout:
 *   [0..5]  pose: rx, ry, rz, tx, ty, tz (Lie algebra, stored as Pose3)
 *   [6..11] velocity: angular_x, angular_y, angular_z, linear_x, linear_y, linear_z
 *   [12..14] accelerometer bias: bias_ax, bias_ay, bias_az
 *
 * Predict: constant-velocity model with bias-corrected acceleration.
 *   pose = pose * Expmap(velocity * dt)
 *   velocity = velocity (unchanged by predict; updated by measurements)
 *   bias = bias (random walk, grows with process noise)
 *
 * Update: full Kalman gain with 15x15 covariance. Cross-covariance between
 * pose, velocity, and bias allows pose corrections to adjust velocity and bias,
 * and vice versa.
 *
 * The accel bias is internal — the EKFModelPlugin interface (updatePose,
 * updateTwist) is unchanged. Bias subtraction happens inside fuseImuSource
 * in eidos_transform_node.cpp when processing linear acceleration.
 */
class HolonomicEKF : public EKFModelPlugin
{
public:
  HolonomicEKF() = default;
  ~HolonomicEKF() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  void predict(double dt) override;

  void updatePose(
    const gtsam::Pose3 & meas,
    const std::array<bool, 6> & mask,
    const gtsam::Vector6 & noise) override;

  void updateTwist(
    const gtsam::Vector6 & meas,
    const std::array<bool, 6> & mask,
    const gtsam::Vector6 & noise) override;

  void updateAcceleration(
    const Eigen::Vector3d & accel, const Eigen::Vector3d & noise, double dt) override;

  gtsam::Pose3 pose() const override;
  gtsam::Vector6 velocity() const override;
  void reset(const gtsam::Pose3 & initial) override;
  Eigen::Vector3d accelBias() const override { return accel_bias_; }
  StateSnapshot snapshot(double time) const override;
  void restore(const StateSnapshot & snap) override;

  /// @brief State dimension.
  static constexpr int kStateDim = 15;

private:
  // ---- State ----
  gtsam::Pose3 pose_;
  gtsam::Vector6 velocity_ = gtsam::Vector6::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();

  // ---- Full 15x15 covariance ----
  Eigen::Matrix<double, kStateDim, kStateDim> P_ =
    Eigen::Matrix<double, kStateDim, kStateDim>::Identity();

  // ---- Process noise (15x15 diagonal) ----
  Eigen::Matrix<double, kStateDim, kStateDim> Q_ =
    Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
};

}  // namespace eidos_transform
