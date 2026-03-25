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

#include <geometry_msgs/msg/twist_stamped.hpp>

#include <mutex>

#include "eidos/plugins/base_motion_model_plugin.hpp"

namespace eidos
{

/**
 * @brief Ackermann kinematic state transition model.
 *
 * Subscribes to TwistStamped (linear.x = speed, angular.z = steering angle)
 * and uses ackermann kinematics to predict the next pose:
 *   dx   = speed * dt
 *   dyaw = speed * tan(steering_angle) / wheelbase * dt
 *
 * When no data is available, prediction returns the current pose (zero velocity).
 *
 * TODO: Switch to ackermann_msgs/AckermannDriveStamped when the CAN state
 * estimator publishes it.
 */
class AckermannMotionModel : public MotionModelPlugin
{
public:
  AckermannMotionModel() = default;
  ~AckermannMotionModel() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  gtsam::Pose3 predict(const gtsam::Pose3 & current, double dt) override;
  bool isReady() const override;

private:
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

  std::mutex data_mtx_;
  double speed_ = 0.0;
  double steering_angle_ = 0.0;
  bool has_data_ = false;

  double wheelbase_ = 2.57;
};

}  // namespace eidos
