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

#include "eidos_transform/holonomic_ekf.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace eidos_transform
{

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void HolonomicEKF::onInitialize()
{
  // Read process noise from parameters (12 values: 6 pose + 6 velocity).
  // Default: small noise for pose, moderate for velocity.
  std::vector<double> process_noise;
  node_->declare_parameter<std::vector<double>>(
    name_ + ".process_noise",
    {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,
     1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2});
  node_->get_parameter(name_ + ".process_noise", process_noise);

  if (process_noise.size() != 12) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] process_noise should have 12 entries, got %zu. Using defaults.",
      name_.c_str(), process_noise.size());
    process_noise = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,
                     1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
  }

  Q_ = Eigen::Matrix<double, 12, 12>::Zero();
  for (int i = 0; i < 12; ++i) {
    Q_(i, i) = process_noise[static_cast<size_t>(i)];
  }

  // Initialize state to identity pose and zero velocity.
  reset(gtsam::Pose3::Identity());

  RCLCPP_INFO(node_->get_logger(), "[%s] HolonomicEKF initialized.", name_.c_str());
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
  if (dt <= 0.0) {
    return;
  }

  // Constant-velocity model:
  //   pose = pose * Expmap(velocity * dt)
  //   velocity = velocity (unchanged)
  gtsam::Vector6 delta = velocity_ * dt;
  if (delta.allFinite() && delta.tail<3>().norm() < 5.0) {
    pose_ = pose_.compose(gtsam::Pose3::Expmap(delta));
  }

  // Store dt for velocity inference in updatePose
  last_pose_update_dt_ = dt;

  // Covariance growth
  P_ += Q_ * dt;
}

// ---------------------------------------------------------------------------
// Update Pose
// ---------------------------------------------------------------------------

void HolonomicEKF::updatePose(
  const gtsam::Pose3 & meas,
  const std::array<bool, 6> & mask,
  const gtsam::Vector6 & noise)
{
  // Compute innovation in Lie algebra: error = Logmap(pose^{-1} * meas)
  // This gives the 6-DOF error as [rx, ry, rz, tx, ty, tz].
  gtsam::Vector6 innovation = gtsam::Pose3::Logmap(pose_.inverse().compose(meas));

  // Apply masked scalar Kalman update for each enabled DOF.
  // For DOF i (in the pose block, indices 0..5 of the state):
  //   H = e_i (selects row i of the 12-state)
  //   K_i = P(i,i) / (P(i,i) + R_i)
  //   x_i += K_i * innovation_i
  //   P(i,i) *= (1 - K_i)
  gtsam::Vector6 pose_correction = gtsam::Vector6::Zero();

  for (int i = 0; i < 6; ++i) {
    if (!mask[static_cast<size_t>(i)]) {
      continue;
    }

    double R_i = noise(i) * noise(i);  // variance = sigma^2
    double S_i = P_(i, i) + R_i;

    if (S_i < 1e-15) {
      continue;
    }

    double K_i = P_(i, i) / S_i;
    pose_correction(i) = K_i * innovation(i);
    P_(i, i) *= (1.0 - K_i);
  }

  // Apply the correction to the pose.
  auto prev_pose = pose_;
  pose_ = pose_.compose(gtsam::Pose3::Expmap(pose_correction));

  // Infer velocity from the pose correction.
  // If the pose jumped by correction, the implied velocity should reflect that.
  // Use a simple exponential blend: velocity = alpha * implied + (1-alpha) * old
  if (has_last_pose_update_) {
    double dt = last_pose_update_dt_;
    if (dt > 0.01 && dt < 1.0) {
      gtsam::Vector6 implied_vel = gtsam::Pose3::Logmap(last_pose_at_update_.between(pose_)) / dt;
      if (implied_vel.allFinite() && implied_vel.tail<3>().norm() < 100.0) {
        constexpr double alpha = 0.3;
        velocity_ = alpha * implied_vel + (1.0 - alpha) * velocity_;
      }
    }
  }
  last_pose_at_update_ = pose_;
  has_last_pose_update_ = true;
}

// ---------------------------------------------------------------------------
// Update Twist
// ---------------------------------------------------------------------------

void HolonomicEKF::updateTwist(
  const gtsam::Vector6 & meas,
  const std::array<bool, 6> & mask,
  const gtsam::Vector6 & noise)
{
  // Innovation is simply (measurement - current velocity).
  gtsam::Vector6 innovation = meas - velocity_;

  // Masked scalar Kalman update on velocity DOFs (indices 6..11 of state).
  for (int i = 0; i < 6; ++i) {
    if (!mask[static_cast<size_t>(i)]) {
      continue;
    }

    int state_idx = i + 6;  // velocity lives in rows 6..11 of the 12-state
    double R_i = noise(i) * noise(i);
    double S_i = P_(state_idx, state_idx) + R_i;

    if (S_i < 1e-15) {
      continue;
    }

    double K_i = P_(state_idx, state_idx) / S_i;
    velocity_(i) += K_i * innovation(i);
    P_(state_idx, state_idx) *= (1.0 - K_i);
  }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

gtsam::Pose3 HolonomicEKF::pose() const
{
  return pose_;
}

gtsam::Vector6 HolonomicEKF::velocity() const
{
  return velocity_;
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void HolonomicEKF::reset(const gtsam::Pose3 & initial)
{
  pose_ = initial;
  velocity_ = gtsam::Vector6::Zero();
  P_ = Eigen::Matrix<double, 12, 12>::Identity() * 1.0;
}

}  // namespace eidos_transform

PLUGINLIB_EXPORT_CLASS(eidos_transform::HolonomicEKF, eidos_transform::EKFModelPlugin)
