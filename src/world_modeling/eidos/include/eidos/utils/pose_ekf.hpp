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

#include <gtsam/geometry/Pose3.h>

#include <Eigen/Dense>

namespace eidos
{

/**
 * @brief Simple 6-DOF pose EKF for fusing high-rate motion model predictions
 * with lower-rate odometry corrections (e.g. LISO scan matching).
 *
 * State: Pose3 in the odom frame.
 * Prediction: relative pose delta from motion model at IMU rate.
 * Measurement: absolute odom-frame pose from a factor plugin at LiDAR rate.
 *
 * Uses Lie algebra (se(3)) for the error state. Corrections are applied
 * smoothly via the Kalman gain — no discontinuities in the output.
 *
 * The covariance is tracked as a 6x6 diagonal matrix [rot3, trans3].
 * This is a simplification (ignores cross-correlations) but sufficient
 * for smooth fusion without jumps.
 */
class PoseEKF
{
public:
  /**
   * @brief Configure the EKF noise parameters.
   * @param process_noise_diag  Per-step process noise [roll,pitch,yaw,x,y,z] variance.
   *                            Grows each prediction step. Higher = trusts MM less.
   * @param measurement_noise_diag  Measurement noise [roll,pitch,yaw,x,y,z] variance.
   *                                Fixed per LISO update. Lower = trusts LISO more.
   */
  void configure(
    const Eigen::Matrix<double, 6, 1> & process_noise_diag, const Eigen::Matrix<double, 6, 1> & measurement_noise_diag);

  /// Reset the filter to a given pose with high initial uncertainty.
  void reset(const gtsam::Pose3 & initial_pose);

  /**
   * @brief Prediction step — apply motion model delta.
   * Called at IMU rate (500 Hz). Grows covariance by Q.
   * @param delta Relative pose change since last prediction.
   */
  void predict(const gtsam::Pose3 & delta);

  /**
   * @brief Measurement update — correct with odom source reading.
   * Called when odom_source (e.g. LISO) produces a new pose.
   * Applies correction smoothly via Kalman gain.
   * @param measurement Absolute odom-frame pose from the odom source.
   */
  void update(const gtsam::Pose3 & measurement);

  /// Get the current fused odom-frame pose. Always smooth.
  const gtsam::Pose3 & pose() const
  {
    return state_;
  }

  bool initialized() const
  {
    return initialized_;
  }

private:
  gtsam::Pose3 state_;
  Eigen::Matrix<double, 6, 6> P_ = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> Q_ = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
  Eigen::Matrix<double, 6, 6> R_ = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;
  bool initialized_ = false;
};

}  // namespace eidos
