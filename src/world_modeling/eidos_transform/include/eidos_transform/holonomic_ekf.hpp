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
 * @brief 12-state holonomic EKF: pose (6-DOF) + body-frame velocity (6-DOF).
 *
 * State vector layout (conceptual):
 *   x = [pose(6), velocity(6)]
 *
 * The pose is stored as a gtsam::Pose3 (not as a vector) to avoid
 * singularities. The covariance P_ is 12x12 diagonal approximation.
 *
 * Predict: constant-velocity model.
 *   pose = pose.compose(Pose3::Expmap(velocity * dt))
 *   velocity = velocity  (unchanged)
 *
 * Update: masked Kalman update on individual DOFs.
 */
class HolonomicEKF : public EKFModelPlugin
{
public:
  HolonomicEKF() = default;
  ~HolonomicEKF() override = default;

  // ---- Lifecycle ----
  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  // ---- EKF interface ----
  void predict(double dt) override;

  void updatePose(const gtsam::Pose3 & meas, const std::array<bool, 6> & mask, const gtsam::Vector6 & noise) override;

  void updateTwist(
    const gtsam::Vector6 & meas, const std::array<bool, 6> & mask, const gtsam::Vector6 & noise) override;

  gtsam::Pose3 pose() const override;
  gtsam::Vector6 velocity() const override;

  void reset(const gtsam::Pose3 & initial) override;

private:
  // ---- State ----
  gtsam::Pose3 pose_;
  gtsam::Vector6 velocity_ = gtsam::Vector6::Zero();

  // ---- Velocity inference from pose updates ----
  gtsam::Pose3 last_pose_at_update_;
  double last_pose_update_dt_ = 0.0;
  bool has_last_pose_update_ = false;

  // ---- Covariance (12x12 diagonal approximation) ----
  Eigen::Matrix<double, 12, 12> P_ = Eigen::Matrix<double, 12, 12>::Identity();

  // ---- Process noise (12x12 diagonal) ----
  Eigen::Matrix<double, 12, 12> Q_ = Eigen::Matrix<double, 12, 12>::Identity();
};

}  // namespace eidos_transform
