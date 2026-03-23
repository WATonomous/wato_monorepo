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

#include "eidos/utils/pose_ekf.hpp"

#include <gtsam/base/Lie.h>

namespace eidos
{

void PoseEKF::configure(
  const Eigen::Matrix<double, 6, 1> & process_noise_diag, const Eigen::Matrix<double, 6, 1> & measurement_noise_diag)
{
  Q_ = process_noise_diag.asDiagonal();
  R_ = measurement_noise_diag.asDiagonal();
}

void PoseEKF::reset(const gtsam::Pose3 & initial_pose)
{
  state_ = initial_pose;
  P_ = Eigen::Matrix<double, 6, 6>::Identity() * 1.0;  // high initial uncertainty
  initialized_ = true;
}

void PoseEKF::predict(const gtsam::Pose3 & delta)
{
  if (!initialized_) return;

  // State prediction: compose the relative delta
  state_ = state_.compose(delta);

  // Covariance prediction: P = P + Q
  // (simplified — assumes identity state transition Jacobian,
  //  valid for small deltas at high rate)
  P_ += Q_;
}

void PoseEKF::update(const gtsam::Pose3 & measurement)
{
  if (!initialized_) {
    reset(measurement);
    return;
  }

  // Innovation in Lie algebra: xi = Log(state^{-1} * measurement)
  // This is the "error" between our prediction and the measurement,
  // expressed as a 6-vector in se(3) [rot3, trans3].
  gtsam::Pose3 error_pose = state_.between(measurement);
  gtsam::Vector6 xi = gtsam::Pose3::Logmap(error_pose);

  // Kalman gain: K = P * (P + R)^{-1}
  // With diagonal P and R, this is element-wise: K_i = P_i / (P_i + R_i)
  Eigen::Matrix<double, 6, 6> S = P_ + R_;
  Eigen::Matrix<double, 6, 6> K = P_ * S.inverse();

  // State correction: state = state * Exp(K * xi)
  gtsam::Vector6 correction = K * xi;
  state_ = state_.compose(gtsam::Pose3::Expmap(correction));

  // Covariance update: P = (I - K) * P
  P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K) * P_;
}

}  // namespace eidos
