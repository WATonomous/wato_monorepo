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

#include <algorithm>  // for std::min, std::max
#include <cmath>  // for std::sin, std::cos, std::atan2
#include <limits>  // for std::numeric_limits
#include <string>  // for std::string
#include <vector>  // for std::vector

#include "rclcpp/rclcpp.hpp"  // Add for rclcpp::Time and Duration

namespace prediction
{

// ============================================================================
// BicycleModel Implementation
// ============================================================================

BicycleModel::BicycleModel()
: max_steering_angle_(M_PI / 4.0)  // 45 degrees
, wheelbase_(2.5)  // Default wheelbase
{}

KinematicState BicycleModel::propagate(const KinematicState & initial_state, double dt)
{
  KinematicState next_state;

  // Bicycle kinematics equations
  // x_dot = v * cos(theta)
  // y_dot = v * sin(theta)
  // theta_dot = v * tan(delta) / L

  next_state.x = initial_state.x + initial_state.v * std::cos(initial_state.theta) * dt;
  next_state.y = initial_state.y + initial_state.v * std::sin(initial_state.theta) * dt;
  next_state.theta = initial_state.theta + initial_state.v * std::tan(initial_state.delta) / wheelbase_ * dt;

  // Ensures deceleration before sharp corner
  double curvature = std::abs(std::tan(initial_state.delta) / wheelbase_);
  const double max_lateral_a = 4.0;  // default, TBD
  const double speed_limit = 15.0;
  const double max_a = 2.0;  // max acceleration
  const double max_d = 4.0;  // max deceleration

  double max_safe_velocity = std::sqrt(max_lateral_a / std::max(curvature, 1e-6));
  double target_v = std::min(max_safe_velocity, speed_limit);

  if (target_v > initial_state.v) {  // accelerate towards target
    next_state.a = max_a;
    next_state.v = initial_state.v + max_a * dt;
  } else if (target_v < initial_state.v) {  // decelerate towards target
    next_state.a = -max_d;
    next_state.v = initial_state.v - max_d * dt;
  } else {  // maintain velocity
    next_state.a = 0.0;
    next_state.v = initial_state.v;
  }

  if (next_state.a > 0.0) {  // Ensure target velocity is not exceeded
    next_state.v = std::min(next_state.v, target_v);
  } else if (next_state.a < 0.0) {  // Ensure target velocity is not undershot
    next_state.v = std::max(next_state.v, target_v);
  }

  next_state.v = std::max(0.0, next_state.v);  // Prevent negative velocity
  next_state.delta = initial_state.delta;

  return next_state;
}

std::vector<geometry_msgs::msg::PoseStamped> BicycleModel::generateTrajectory(
  const KinematicState & initial_state,
  const std::vector<Eigen::Vector2d> & path_points,
  double horizon,
  double dt,
  const rclcpp::Time & start_time,
  const std::string & frame_id)
{
  std::vector<geometry_msgs::msg::PoseStamped> trajectory;

  // If no path provided, return empty trajectory
  if (path_points.empty()) {
    return trajectory;
  }

  KinematicState current_state = initial_state;
  double t = dt;
  const double lookahead_distance = 3.0;

  while (t <= horizon) {
    // compute heading with pure pursuit logic
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < path_points.size(); ++i) {
      double dx = path_points[i].x() - current_state.x;
      double dy = path_points[i].y() - current_state.y;
      double distance = std::sqrt(dx * dx + dy * dy);  // finding Euclidean distance

      if (distance < min_distance) {
        min_distance = distance;
        closest_idx = i;
      }
    }

    // Find lookahead point along path
    size_t lookahead_idx = closest_idx;
    double accumulated_distance = 0.0;

    for (size_t i = closest_idx; i < path_points.size() - 1; ++i) {
      double dx = path_points[i + 1].x() - path_points[i].x();
      double dy = path_points[i + 1].y() - path_points[i].y();
      double segment_distance = std::sqrt(dx * dx + dy * dy);
      accumulated_distance += segment_distance;

      if (accumulated_distance >= lookahead_distance) {
        lookahead_idx = i + 1;
        break;
      }
    }

    // Use the last point if lookahead exceeds the end of the path
    if (lookahead_idx >= path_points.size()) {
      lookahead_idx = path_points.size() - 1;
    }

    // Compute steering angle
    double dx = path_points[lookahead_idx].x() - current_state.x;
    double dy = path_points[lookahead_idx].y() - current_state.y;
    double ld = std::sqrt(dx * dx + dy * dy);  // Euclidean lookahead distance

    if (ld < 1e-6) {
      ld = 1e-6;
    }

    double alpha = std::atan2(dy, dx) - current_state.theta;

    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;

    current_state.delta = std::atan2(2.0 * wheelbase_ * std::sin(alpha), ld);
    current_state.delta = std::max(
      -max_steering_angle_, std::min(max_steering_angle_, current_state.delta));  // clamp steering angle to limits

    // propagate state by dt in seconds
    current_state = propagate(current_state, dt);

    // create PoseStamped for current state
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = start_time + rclcpp::Duration::from_seconds(t);

    pose_stamped.pose.position.x = current_state.x;
    pose_stamped.pose.position.y = current_state.y;
    pose_stamped.pose.position.z = 0.0;

    pose_stamped.pose.orientation.w = std::cos(current_state.theta / 2.0);
    pose_stamped.pose.orientation.x = 0.0;
    pose_stamped.pose.orientation.y = 0.0;
    pose_stamped.pose.orientation.z = std::sin(current_state.theta / 2.0);

    trajectory.push_back(pose_stamped);
    t += dt;
  }

  return trajectory;
}

// ============================================================================
// ConstantVelocityModel Implementation
// ============================================================================

ConstantVelocityModel::ConstantVelocityModel()
: position_noise_std_(0.1)
, heading_noise_std_(0.05)
{}

KinematicState ConstantVelocityModel::propagate(const KinematicState & initial_state, double dt)
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

std::vector<geometry_msgs::msg::PoseStamped> ConstantVelocityModel::generateTrajectory(
  const KinematicState & initial_state,
  double horizon,
  double dt,
  bool add_noise,
  const rclcpp::Time & start_time,
  const std::string & frame_id)
{
  std::vector<geometry_msgs::msg::PoseStamped> trajectory;

  KinematicState current_state = initial_state;
  double t = dt;

  while (t <= horizon) {
    current_state = propagate(current_state, dt);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = start_time + rclcpp::Duration::from_seconds(t);

    pose_stamped.pose.position.x = current_state.x;
    pose_stamped.pose.position.y = current_state.y;
    pose_stamped.pose.position.z = 0.0;

    // Convert heading to quaternion
    pose_stamped.pose.orientation.w = std::cos(current_state.theta / 2.0);
    pose_stamped.pose.orientation.x = 0.0;
    pose_stamped.pose.orientation.y = 0.0;
    pose_stamped.pose.orientation.z = std::sin(current_state.theta / 2.0);

    trajectory.push_back(pose_stamped);
    t += dt;
  }

  return trajectory;
}

}  // namespace prediction
