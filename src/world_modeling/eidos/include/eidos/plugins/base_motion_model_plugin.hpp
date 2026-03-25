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
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <tf2_ros/buffer.h>

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "eidos/utils/types.hpp"

namespace eidos
{

/**
 * @brief Base class for motion model plugins.
 *
 * A motion model is a kinematic state transition model. It predicts the next
 * pose given the current pose and a time step. TransformManager calls predict()
 * at 500Hz for smooth odom->base_link TF broadcasting.
 *
 * The motion model does NOT subscribe to raw sensor data — sensor measurements
 * come from factor plugins (LISO, GPS, ImuFactor). The motion model's job is
 * to provide the prediction step of the EKF: "given where I am now, where will
 * I be in dt seconds?"
 *
 * After each measurement correction, TransformManager calls
 * onMeasurementUpdate() so the model can update its internal velocity estimate
 * from the corrected pose.
 */
class MotionModelPlugin
{
public:
  virtual ~MotionModelPlugin() = default;

  const std::string & getName() const { return name_; }

  void initialize(
    const std::string & name,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    tf2_ros::Buffer * tf,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    const std::atomic<SlamState> * state)
  {
    name_ = name;
    node_ = node;
    tf_ = tf;
    callback_group_ = callback_group;
    state_ = state;
    onInitialize();
  }

  // ---- Lifecycle ----
  virtual void onInitialize() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  /// Predict the next pose from the current pose after dt seconds.
  /// Called by TransformManager at high rate for smooth odom->base TF.
  virtual gtsam::Pose3 predict(const gtsam::Pose3 & current, double dt) = 0;

  /// Called after each measurement correction so the model can update its
  /// internal velocity estimate from consecutive corrected poses.
  virtual void onMeasurementUpdate(const gtsam::Pose3 & corrected, double timestamp)
  {
    (void)corrected;
    (void)timestamp;
  }

  /// Produce a BetweenFactor bridging two temporally consecutive states
  /// created by different factor plugins. Integrates the kinematic model
  /// over [ts_from, ts_to]. Returns nullptr if unable.
  virtual gtsam::NonlinearFactor::shared_ptr getBetweenFactor(
    gtsam::Key key_from, double ts_from,
    gtsam::Key key_to, double ts_to)
  {
    (void)key_from;
    (void)ts_from;
    (void)key_to;
    (void)ts_to;
    return nullptr;
  }

  /// Whether the motion model is ready. Default: true.
  virtual bool isReady() const { return true; }

  virtual std::string getReadyStatus() const { return ""; }

protected:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  tf2_ros::Buffer * tf_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  const std::atomic<SlamState> * state_ = nullptr;
};

}  // namespace eidos
