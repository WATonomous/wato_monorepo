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

/**
 * @file motion_models.hpp
 * @brief Physics-based motion models for trajectory propagation.
 *
 * Provides kinematic models to propagate object state forward in time:
 * - BicycleModel: Vehicle motion with front-wheel steering
 * - ConstantVelocityModel: Simple linear motion with optional noise
 */

#ifndef PREDICTION__MOTION_MODELS_HPP_
#define PREDICTION__MOTION_MODELS_HPP_

#include <Eigen/Dense>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"

namespace prediction
{

/**
 * @brief State vector for kinematic models
 */
struct KinematicState
{
  double x;  // Position x
  double y;  // Position y
  double theta;  // Heading angle
  double v;  // Velocity
  double a;  // Acceleration (optional)
  double delta;  // Steering angle (for bicycle model)
};

/**
 * @brief Bicycle kinematic model for vehicle motion
 *
 * Models vehicle motion using bicycle kinematics with front wheel steering.
 */
class BicycleModel
{
public:
  BicycleModel();

  /**
   * @brief Propagate state forward using bicycle model
   * @param initial_state Initial kinematic state
   * @param dt Time step
   * @param wheelbase Vehicle wheelbase
   * @return New kinematic state after dt
   */
  KinematicState propagate(const KinematicState & initial_state, double dt, double wheelbase = 2.5);

  /**
   * @brief Generate trajectory following a path
   * @param initial_state Initial state
   * @param path_points Points along desired path
   * @param horizon Time horizon
   * @param dt Time step
   * @return Vector of poses along trajectory
   */
  std::vector<geometry_msgs::msg::Pose> generateTrajectory(
    const KinematicState & initial_state, const std::vector<Eigen::Vector2d> & path_points, double horizon, double dt);

private:
  double max_steering_angle_;  // Maximum steering angle (rad)
};

/**
 * @brief Constant velocity model with Gaussian noise
 *
 * Simple motion model assuming constant velocity with additive noise.
 */
class ConstantVelocityModel
{
public:
  ConstantVelocityModel();

  /**
   * @brief Propagate state forward with constant velocity
   * @param initial_state Initial kinematic state
   * @param dt Time step
   * @return New kinematic state after dt
   */
  KinematicState propagate(const KinematicState & initial_state, double dt);

  /**
   * @brief Generate trajectory with constant velocity
   * @param initial_state Initial state
   * @param horizon Time horizon
   * @param dt Time step
   * @param add_noise Whether to add Gaussian noise
   * @return Vector of poses along trajectory
   */
  std::vector<geometry_msgs::msg::Pose> generateTrajectory(
    const KinematicState & initial_state, double horizon, double dt, bool add_noise = false);

private:
  double position_noise_std_;  // Standard deviation for position noise
  double heading_noise_std_;  // Standard deviation for heading noise
};

}  // namespace prediction

#endif  // PREDICTION__MOTION_MODELS_HPP_
