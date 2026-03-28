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

#include "eidos_transform/plugins/holonomic_ekf.hpp"

#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace eidos_transform
{

static constexpr int N = HolonomicEKF::kStateDim;  // 15

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void HolonomicEKF::onInitialize()
{
  // Process noise: 15 diagonal values
  //   [0..5]   pose process noise
  //   [6..11]  velocity process noise
  //   [12..14] accel bias random walk noise
  std::vector<double> pn;
  node_->declare_parameter<std::vector<double>>(
    name_ + ".process_noise",
    {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,
     1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2,
     1e-6, 1e-6, 1e-6});
  node_->get_parameter(name_ + ".process_noise", pn);

  if (pn.size() != static_cast<size_t>(N)) {
    RCLCPP_WARN(
      node_->get_logger(), "[%s] process_noise should have %d entries, got %zu. Using defaults.",
      name_.c_str(), N, pn.size());
    pn = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,
          1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2,
          1e-6, 1e-6, 1e-6};
  }

  Q_ = Eigen::Matrix<double, N, N>::Zero();
  for (int i = 0; i < N; ++i) {
    Q_(i, i) = pn[static_cast<size_t>(i)];
  }

  pose_ = gtsam::Pose3::Identity();
  velocity_ = gtsam::Vector6::Zero();
  accel_bias_ = Eigen::Vector3d::Zero();
  P_ = Eigen::Matrix<double, N, N>::Identity();

  RCLCPP_INFO(node_->get_logger(), "[%s] HolonomicEKF initialized (15-state).", name_.c_str());
}

void HolonomicEKF::activate()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] HolonomicEKF activated.", name_.c_str());
}

void HolonomicEKF::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] HolonomicEKF deactivated.", name_.c_str());
}

// ---------------------------------------------------------------------------
// Predict
// ---------------------------------------------------------------------------

void HolonomicEKF::predict(double dt)
{
  if (dt <= 0.0) return;

  // Constant-velocity model:
  //   pose = pose * Expmap(velocity * dt)
  //   velocity = velocity (unchanged)
  //   bias = bias (random walk)
  gtsam::Vector6 delta = velocity_ * dt;
  if (delta.allFinite() && delta.tail<3>().norm() < 5.0) {
    pose_ = pose_.compose(gtsam::Pose3::Expmap(delta));
  }

  // State transition Jacobian F (15x15):
  //   F = [I  dt*I  0]   (pose depends on velocity)
  //       [0    I   0]   (velocity unchanged)
  //       [0    0   I]   (bias random walk)
  Eigen::Matrix<double, N, N> F = Eigen::Matrix<double, N, N>::Identity();
  for (int i = 0; i < 6; ++i) {
    F(i, i + 6) = dt;  // pose row depends on velocity
  }

  // Covariance propagation
  P_ = F * P_ * F.transpose() + Q_ * dt;
}

// ---------------------------------------------------------------------------
// Update Pose — full Kalman gain (15x1)
// ---------------------------------------------------------------------------

void HolonomicEKF::updatePose(
  const gtsam::Pose3 & meas, const std::array<bool, 6> & mask, const gtsam::Vector6 & noise)
{
  gtsam::Vector6 innovation = gtsam::Pose3::Logmap(pose_.inverse().compose(meas));

  for (int i = 0; i < 6; ++i) {
    if (!mask[static_cast<size_t>(i)]) continue;

    double R_i = noise(i) * noise(i);
    double S_i = P_(i, i) + R_i;
    if (S_i < 1e-15) continue;

    // Full 15x1 Kalman gain
    Eigen::Matrix<double, N, 1> K = P_.col(i) / S_i;
    double innov_i = innovation(i);

    // Correct pose (states 0..5)
    gtsam::Vector6 pose_delta = K.segment<6>(0) * innov_i;
    pose_ = pose_.compose(gtsam::Pose3::Expmap(pose_delta));

    // Correct velocity (states 6..11) via cross-covariance
    velocity_ += K.segment<6>(6) * innov_i;

    // Correct accel bias (states 12..14) via cross-covariance
    accel_bias_ += K.segment<3>(12) * innov_i;

    // Covariance update
    P_ -= K * P_.row(i);
    P_ = (P_ + P_.transpose()) * 0.5;

    // Recompute innovation (pose changed)
    innovation = gtsam::Pose3::Logmap(pose_.inverse().compose(meas));
  }
}

// ---------------------------------------------------------------------------
// Update Twist — full Kalman gain (15x1)
// ---------------------------------------------------------------------------

void HolonomicEKF::updateTwist(
  const gtsam::Vector6 & meas, const std::array<bool, 6> & mask, const gtsam::Vector6 & noise)
{
  gtsam::Vector6 innovation = meas - velocity_;

  for (int i = 0; i < 6; ++i) {
    if (!mask[static_cast<size_t>(i)]) continue;

    int si = i + 6;  // velocity in state indices 6..11
    double R_i = noise(i) * noise(i);
    double S_i = P_(si, si) + R_i;
    if (S_i < 1e-15) continue;

    // Full 15x1 Kalman gain
    Eigen::Matrix<double, N, 1> K = P_.col(si) / S_i;
    double innov_i = innovation(i);

    // Correct pose via cross-covariance
    gtsam::Vector6 pose_delta = K.segment<6>(0) * innov_i;
    pose_ = pose_.compose(gtsam::Pose3::Expmap(pose_delta));

    // Correct velocity
    velocity_ += K.segment<6>(6) * innov_i;

    // Correct accel bias via cross-covariance
    accel_bias_ += K.segment<3>(12) * innov_i;

    // Covariance update
    P_ -= K * P_.row(si);
    P_ = (P_ + P_.transpose()) * 0.5;

    // Recompute innovation
    innovation = meas - velocity_;
  }
}

// ---------------------------------------------------------------------------
// Update Acceleration — direct observation of (accel - bias)
// ---------------------------------------------------------------------------

void HolonomicEKF::updateAcceleration(
  const Eigen::Vector3d & accel, const Eigen::Vector3d & noise, double dt)
{
  if (dt <= 0.0) return;

  // The acceleration measurement observes: z = true_accel + bias + noise
  // Under constant-velocity, predicted acceleration = 0.
  // Innovation = measured_accel - predicted_accel - bias = accel - bias
  //
  // This acceleration should produce a velocity change of (accel - bias) * dt.
  // We update the velocity states (9,10,11 = linear velocity) and bias states (12,13,14).
  //
  // Observation matrix H for axis i:
  //   H[linear_vel_i] = 1/dt   (accel = dv/dt)
  //   H[bias_i] = 1             (accel includes bias)
  //   All other H entries = 0
  //
  // Innovation: z_i - h(x) = accel_i - (0 + bias_i) = accel_i - bias_i

  for (int i = 0; i < 3; ++i) {
    double innov = accel(i) - accel_bias_(i);
    double R_i = noise(i) * noise(i);

    // Build H row (1x15): maps to linear velocity and bias
    Eigen::Matrix<double, 1, N> H = Eigen::Matrix<double, 1, N>::Zero();
    int vel_idx = 9 + i;   // linear velocity: states 9, 10, 11
    int bias_idx = 12 + i;  // bias: states 12, 13, 14
    H(0, vel_idx) = 1.0 / dt;
    H(0, bias_idx) = 1.0;

    // S = H * P * H^T + R
    double S = (H * P_ * H.transpose())(0, 0) + R_i;
    if (S < 1e-15) continue;

    // K = P * H^T / S (15x1)
    Eigen::Matrix<double, N, 1> K = P_ * H.transpose() / S;

    // State correction
    gtsam::Vector6 pose_delta = K.segment<6>(0) * innov;
    pose_ = pose_.compose(gtsam::Pose3::Expmap(pose_delta));
    velocity_ += K.segment<6>(6) * innov;
    accel_bias_ += K.segment<3>(12) * innov;

    // Covariance update: P = (I - K*H) * P
    P_ = (Eigen::Matrix<double, N, N>::Identity() - K * H) * P_;
    P_ = (P_ + P_.transpose()) * 0.5;
  }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

gtsam::Pose3 HolonomicEKF::pose() const { return pose_; }
gtsam::Vector6 HolonomicEKF::velocity() const { return velocity_; }

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void HolonomicEKF::reset(const gtsam::Pose3 & initial)
{
  pose_ = initial;
  velocity_ = gtsam::Vector6::Zero();
  accel_bias_ = Eigen::Vector3d::Zero();
  P_ = Eigen::Matrix<double, N, N>::Identity();
}

// ---------------------------------------------------------------------------
// Snapshot / Restore
// ---------------------------------------------------------------------------

StateSnapshot HolonomicEKF::snapshot(double time) const
{
  StateSnapshot snap;
  snap.time = time;
  snap.pose = pose_;
  snap.velocity = velocity_;
  snap.accel_bias = accel_bias_;
  snap.P = P_;
  return snap;
}

void HolonomicEKF::restore(const StateSnapshot & snap)
{
  pose_ = snap.pose;
  velocity_ = snap.velocity;
  accel_bias_ = snap.accel_bias;
  if (snap.P.rows() == kStateDim && snap.P.cols() == kStateDim) {
    P_ = snap.P;
  }
}

}  // namespace eidos_transform

PLUGINLIB_EXPORT_CLASS(eidos_transform::HolonomicEKF, eidos_transform::EKFModelPlugin)
