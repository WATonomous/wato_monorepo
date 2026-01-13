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

#include "prediction/motion_models.hpp"
#include <cmath>

namespace prediction
{

// ============================================================================
// BicycleModel Implementation
// ============================================================================

BicycleModel::BicycleModel()
: max_steering_angle_(M_PI / 4.0)  // 45 degrees
{
}

KinematicState BicycleModel::propagate(
  const KinematicState & initial_state,
  double dt,
  double wheelbase)
{
  KinematicState next_state;

  // Bicycle kinematics equations
  // x_dot = v * cos(theta)
  // y_dot = v * sin(theta)
  // theta_dot = v * tan(delta) / L
  
  next_state.x = initial_state.x + initial_state.v * std::cos(initial_state.theta) * dt;
  next_state.y = initial_state.y + initial_state.v * std::sin(initial_state.theta) * dt;
  next_state.theta = initial_state.theta + 
                     initial_state.v * std::tan(initial_state.delta) / wheelbase * dt;
  next_state.v = initial_state.v;  // Constant velocity for now
  next_state.delta = initial_state.delta;
  next_state.a = initial_state.a;

  return next_state;
}

std::vector<geometry_msgs::msg::Pose> BicycleModel::generateTrajectory(
  const KinematicState & initial_state,
  const std::vector<Eigen::Vector2d> & path_points,
  double horizon,
  double dt)
{
  // ============================================================================
  // JOHN TASK: BICYCLE MODEL PATH FOLLOWING
  // ============================================================================
  // TODO: Implement path following controller (pure pursuit or Stanley)
  // - Follow path_points (lanelet centerline) using bicycle kinematics
  // - Use propagate() function for state propagation
  // - Apply steering constraints (max_steering_angle_)
  // - Return std::vector<geometry_msgs::msg::Pose>
  // ============================================================================
  
  std::vector<geometry_msgs::msg::Pose> trajectory;
  
  // TODO: Implement path following with bicycle model
  // 1. For each time step, compute desired steering angle to follow path
  // 2. Apply steering constraints
  // 3. Propagate state
  // 4. Convert to Pose message
  
  KinematicState current_state = initial_state;
  double t = 0.0;
  
  while (t < horizon) {
    // TODO: Compute steering angle to follow path
    // For now, just propagate with current steering
    current_state = propagate(current_state, dt);
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = current_state.x;
    pose.position.y = current_state.y;
    pose.position.z = 0.0;
    
    // Convert heading to quaternion
    pose.orientation.w = std::cos(current_state.theta / 2.0);
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(current_state.theta / 2.0);
    
    trajectory.push_back(pose);
    t += dt;
  }
  
  return trajectory;
}

// ============================================================================
// ConstantVelocityModel Implementation
// ============================================================================

ConstantVelocityModel::ConstantVelocityModel()
: position_noise_std_(0.1),
  heading_noise_std_(0.05)
{
}

KinematicState ConstantVelocityModel::propagate(
  const KinematicState & initial_state,
  double dt)
{
  KinematicState next_state;

  // Constant velocity propagation
  next_state.x = initial_state.x + initial_state.v * std::cos(initial_state.theta) * dt;
  next_state.y = initial_state.y + initial_state.v * std::sin(initial_state.theta) * dt;
  next_state.theta = initial_state.theta;
  next_state.v = initial_state.v;
  next_state.a = 0.0;
  next_state.delta = 0.0;

  return next_state;
}

std::vector<geometry_msgs::msg::Pose> ConstantVelocityModel::generateTrajectory(
  const KinematicState & initial_state,
  double horizon,
  double dt,
  bool add_noise)
{
  // ============================================================================
  // GIRISH TASK: CONSTANT VELOCITY MODEL WITH NOISE
  // ============================================================================
  // TODO: Add Gaussian noise when add_noise == true
  // - Use position_noise_std_ and heading_noise_std_
  // - Generate multiple samples for uncertainty representation
  // ============================================================================
  
  std::vector<geometry_msgs::msg::Pose> trajectory;
  
  KinematicState current_state = initial_state;
  double t = 0.0;
  
  while (t < horizon) {
    current_state = propagate(current_state, dt);
    
    // TODO: Add Gaussian noise if requested
    if (add_noise) {
      // current_state.x += sample_gaussian(0, position_noise_std_);
      // current_state.y += sample_gaussian(0, position_noise_std_);
      // current_state.theta += sample_gaussian(0, heading_noise_std_);
    }
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = current_state.x;
    pose.position.y = current_state.y;
    pose.position.z = 0.0;
    
    // Convert heading to quaternion
    pose.orientation.w = std::cos(current_state.theta / 2.0);
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(current_state.theta / 2.0);
    
    trajectory.push_back(pose);
    t += dt;
  }
  
  return trajectory;
}

}  // namespace prediction
