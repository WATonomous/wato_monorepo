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

#include "eidos/plugins/motion_models/holonomic_motion_model.hpp"

#include <gtsam/base/Lie.h>

#include <pluginlib/class_list_macros.hpp>

namespace eidos
{

void HolonomicMotionModel::onInitialize()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (holonomic constant-velocity)", name_.c_str());
}

void HolonomicMotionModel::activate()
{
  velocity_ = gtsam::Vector6::Zero();
  has_last_corrected_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void HolonomicMotionModel::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

gtsam::Pose3 HolonomicMotionModel::predict(const gtsam::Pose3 & current, double dt)
{
  if (dt <= 0.0 || !velocity_.allFinite() || velocity_.norm() < 1e-6) return current;

  gtsam::Vector6 scaled = velocity_ * dt;
  if (!scaled.allFinite() || scaled.tail<3>().norm() > 5.0) return current;  // cap ~5m per tick

  return current.compose(gtsam::Pose3::Expmap(scaled));
}

void HolonomicMotionModel::onMeasurementUpdate(const gtsam::Pose3 & corrected, double timestamp)
{
  if (!has_last_corrected_) {
    last_corrected_ = corrected;
    last_corrected_time_ = timestamp;
    has_last_corrected_ = true;
    return;
  }

  double dt = timestamp - last_corrected_time_;
  if (dt > 0.01 && dt < 1.0) {
    // Estimate velocity from consecutive corrected poses
    gtsam::Pose3 relative = last_corrected_.between(corrected);
    gtsam::Vector6 v = gtsam::Pose3::Logmap(relative) / dt;
    // Guard against NaN/inf and unreasonable velocities (>100 m/s or >10 rad/s)
    if (v.allFinite() && v.head<3>().norm() < 10.0 && v.tail<3>().norm() < 100.0) {
      velocity_ = v;
    }
  }

  last_corrected_ = corrected;
  last_corrected_time_ = timestamp;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::HolonomicMotionModel, eidos::MotionModelPlugin)
