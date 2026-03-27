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

#include <array>
#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace eidos_transform
{

/**
 * @brief Base class for EKF motion model plugins.
 *
 * Provides the interface for predict/update/reset used by the
 * EidosTransformNode tick loop. Loaded at runtime via pluginlib.
 */
class EKFModelPlugin
{
public:
  virtual ~EKFModelPlugin() = default;

  /**
   * @brief Framework entry point: store name and node handle, then call onInitialize().
   * @param name Plugin instance name (used for parameter namespacing).
   * @param node Shared pointer to the parent lifecycle node.
   * @note Called exactly once by EidosTransformNode after pluginlib construction.
   */
  void initialize(const std::string & name, rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  {
    name_ = name;
    node_ = node;
    onInitialize();
  }

  // ---- Lifecycle (pure virtual) ----

  /// @brief Plugin-specific initialization (declare parameters, allocate state).
  virtual void onInitialize() = 0;
  /// @brief Activate the plugin (start accepting measurements).
  virtual void activate() = 0;
  /// @brief Deactivate the plugin (stop accepting measurements).
  virtual void deactivate() = 0;

  // ---- EKF interface ----

  /**
   * @brief Propagate state forward by dt seconds using constant-velocity model.
   * @param dt Time step in seconds.
   */
  virtual void predict(double dt) = 0;

  /**
   * @brief Fuse a pose measurement (6-DOF, masked).
   * @param meas Measured pose in the odom frame.
   * @param mask Which of the 6 DOF (rx, ry, rz, tx, ty, tz) to use.
   * @param noise Standard deviation for each DOF.
   */
  virtual void updatePose(
    const gtsam::Pose3 & meas, const std::array<bool, 6> & mask, const gtsam::Vector6 & noise) = 0;

  /**
   * @brief Fuse a twist measurement (6-DOF body-frame velocity, masked).
   * @param meas Measured twist [angular_x, angular_y, angular_z, linear_x, linear_y, linear_z].
   * @param mask Which of the 6 DOF to use.
   * @param noise Standard deviation for each DOF.
   */
  virtual void updateTwist(
    const gtsam::Vector6 & meas, const std::array<bool, 6> & mask, const gtsam::Vector6 & noise) = 0;

  /**
   * @brief Get the current estimated pose.
   * @return The EKF's current pose estimate in the odom frame.
   */
  virtual gtsam::Pose3 pose() const = 0;

  /**
   * @brief Get the current estimated body-frame velocity (6-DOF).
   * @return gtsam::Vector6 [angular_x, angular_y, angular_z, linear_x, linear_y, linear_z].
   */
  virtual gtsam::Vector6 velocity() const = 0;

  /**
   * @brief Hard-reset the filter to a known pose, zero velocity, and default covariance.
   * @param initial The pose to reset to.
   */
  virtual void reset(const gtsam::Pose3 & initial) = 0;

protected:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace eidos_transform
