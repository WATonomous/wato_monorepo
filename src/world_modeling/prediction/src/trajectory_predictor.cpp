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

#include "prediction/trajectory_predictor.hpp"

#include <cmath>  // for std::atan2, std::sin, std::cos
#include <memory>  // for std::make_unique
#include <string>  // for std::string
#include <vector>  // for std::vector

#include "rclcpp/rclcpp.hpp"  // Add this for rclcpp::Duration

namespace prediction
{

namespace
{
// Extract yaw from quaternion
double extractYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

TrajectoryPredictor::TrajectoryPredictor(
  rclcpp_lifecycle::LifecycleNode * node, double prediction_horizon, double time_step)
: node_(node)
, prediction_horizon_(prediction_horizon)
, time_step_(time_step)
{
  // Initialize motion models
  bicycle_model_ = std::make_unique<BicycleModel>();
  constant_velocity_model_ = std::make_unique<ConstantVelocityModel>();

  RCLCPP_INFO(node_->get_logger(), "TrajectoryPredictor initialized");
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateHypotheses(
  const vision_msgs::msg::Detection3D & detection)
{
  ObjectType obj_type = classifyObjectType(detection);

  std::vector<TrajectoryHypothesis> hypotheses;

  switch (obj_type) {
    case ObjectType::VEHICLE:
      hypotheses = generateVehicleHypotheses(detection);
      break;
    case ObjectType::PEDESTRIAN:
      hypotheses = generatePedestrianHypotheses(detection);
      break;
    case ObjectType::CYCLIST:
      hypotheses = generateCyclistHypotheses(detection);
      break;
    default:
      RCLCPP_WARN(node_->get_logger(), "Unknown object type, skipping prediction");
      break;
  }

  return hypotheses;
}

ObjectType TrajectoryPredictor::classifyObjectType(const vision_msgs::msg::Detection3D & detection)
{
  // PLACEHOLDER: Simple rule-based classification using bounding box dimensions
  RCLCPP_DEBUG(node_->get_logger(), "Classifying object type for detection: %s", detection.id.c_str());

  // Use bounding box size as a simple heuristic
  double length = detection.bbox.size.x;
  double height = detection.bbox.size.z;

  // Vehicle: large length (> 3m)
  if (length > 3.5) {
    return ObjectType::VEHICLE;
  }
  // Pedestrian: small and tall
  else if (length < 1.0 && height > 1.0)
  {
    return ObjectType::PEDESTRIAN;
  }
  // Cyclist: medium size
  else if (length >= 1.0 && length <= 3.5)
  {
    return ObjectType::CYCLIST;
  }

  // Default to vehicle
  return ObjectType::VEHICLE;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateVehicleHypotheses(
  const vision_msgs::msg::Detection3D & detection)
{
  std::vector<TrajectoryHypothesis> hypotheses;

  // Get current time and frame for timestamps
  rclcpp::Time current_time = node_->get_clock()->now();
  std::string frame_id = "map";

  // Extract initial state from detection
  KinematicState initial_state;
  initial_state.x = detection.bbox.center.position.x;
  initial_state.y = detection.bbox.center.position.y;
  initial_state.theta = extractYaw(detection.bbox.center.orientation);

  // Estimate speed from bounding box size heuristic
  double length = detection.bbox.size.x;
  initial_state.v = (length > 3.5) ? 5.0 : 1.4;

  initial_state.a = 0.0;
  initial_state.delta = 0.0;

  // ==== Vehicle Hypothesis: Continue Straight ====
  {
    std::vector<Eigen::Vector2d> straight_path;
    for (double d = 0.0; d <= initial_state.v * prediction_horizon_ + 10.0; d += 1.0) {
      straight_path.emplace_back(
        initial_state.x + d * std::cos(initial_state.theta), initial_state.y + d * std::sin(initial_state.theta));
    }

    TrajectoryHypothesis straight_hyp;
    straight_hyp.header.stamp = current_time;
    straight_hyp.header.frame_id = frame_id;
    straight_hyp.intent = Intent::CONTINUE_STRAIGHT;
    straight_hyp.probability = 0.0;

    straight_hyp.poses = bicycle_model_->generateTrajectory(
      initial_state, straight_path, prediction_horizon_, time_step_, current_time, frame_id);

    hypotheses.push_back(straight_hyp);
  }

  // ==== Vehicle Hypothesis: Turn Left ====
  {
    std::vector<Eigen::Vector2d> left_path;
    double turn_radius = 10.0;
    for (double angle = 0.0; angle <= M_PI / 2.0; angle += 0.1) {
      double path_x = initial_state.x + turn_radius * std::sin(angle) * std::cos(initial_state.theta) -
                      turn_radius * (1.0 - std::cos(angle)) * std::sin(initial_state.theta);
      double path_y = initial_state.y + turn_radius * std::sin(angle) * std::sin(initial_state.theta) +
                      turn_radius * (1.0 - std::cos(angle)) * std::cos(initial_state.theta);
      left_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis left_hyp;
    left_hyp.header.stamp = current_time;
    left_hyp.header.frame_id = frame_id;
    left_hyp.intent = Intent::TURN_LEFT;
    left_hyp.probability = 0.0;

    left_hyp.poses = bicycle_model_->generateTrajectory(
      initial_state, left_path, prediction_horizon_, time_step_, current_time, frame_id);

    hypotheses.push_back(left_hyp);
  }

  // ==== Vehicle Hypothesis: Turn Right ====
  {
    std::vector<Eigen::Vector2d> right_path;
    double turn_radius = 10.0;
    for (double angle = 0.0; angle <= M_PI / 2.0; angle += 0.1) {
      double path_x = initial_state.x + turn_radius * std::sin(angle) * std::cos(initial_state.theta) +
                      turn_radius * (1.0 - std::cos(angle)) * std::sin(initial_state.theta);
      double path_y = initial_state.y + turn_radius * std::sin(angle) * std::sin(initial_state.theta) -
                      turn_radius * (1.0 - std::cos(angle)) * std::cos(initial_state.theta);
      right_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis right_hyp;
    right_hyp.header.stamp = current_time;
    right_hyp.header.frame_id = frame_id;
    right_hyp.intent = Intent::TURN_RIGHT;
    right_hyp.probability = 0.0;

    right_hyp.poses = bicycle_model_->generateTrajectory(
      initial_state, right_path, prediction_horizon_, time_step_, current_time, frame_id);

    hypotheses.push_back(right_hyp);
  }

  // ==== Vehicle Hypothesis: Lane Change Left ====
  {
    std::vector<Eigen::Vector2d> lcl_path;
    double lane_width = 3.5;
    double lc_distance = 30.0;
    for (double d = 0.0; d <= lc_distance; d += 1.0) {
      double lateral_offset = lane_width * (d / lc_distance);
      double path_x =
        initial_state.x + d * std::cos(initial_state.theta) - lateral_offset * std::sin(initial_state.theta);
      double path_y =
        initial_state.y + d * std::sin(initial_state.theta) + lateral_offset * std::cos(initial_state.theta);
      lcl_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis lcl_hyp;
    lcl_hyp.header.stamp = current_time;
    lcl_hyp.header.frame_id = frame_id;
    lcl_hyp.intent = Intent::LANE_CHANGE_LEFT;
    lcl_hyp.probability = 0.0;

    lcl_hyp.poses = bicycle_model_->generateTrajectory(
      initial_state, lcl_path, prediction_horizon_, time_step_, current_time, frame_id);

    hypotheses.push_back(lcl_hyp);
  }

  // ==== Vehicle Hypothesis: Lane Change Right ====
  {
    std::vector<Eigen::Vector2d> lcr_path;
    double lane_width = 3.5;
    double lc_distance = 30.0;
    for (double d = 0.0; d <= lc_distance; d += 1.0) {
      double lateral_offset = lane_width * (d / lc_distance);
      double path_x =
        initial_state.x + d * std::cos(initial_state.theta) + lateral_offset * std::sin(initial_state.theta);
      double path_y =
        initial_state.y + d * std::sin(initial_state.theta) - lateral_offset * std::cos(initial_state.theta);
      lcr_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis lcr_hyp;
    lcr_hyp.header.stamp = current_time;
    lcr_hyp.header.frame_id = frame_id;
    lcr_hyp.intent = Intent::LANE_CHANGE_RIGHT;
    lcr_hyp.probability = 0.0;

    lcr_hyp.poses = bicycle_model_->generateTrajectory(
      initial_state, lcr_path, prediction_horizon_, time_step_, current_time, frame_id);

    hypotheses.push_back(lcr_hyp);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Generated %zu vehicle trajectory hypotheses", hypotheses.size());

  return hypotheses;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generatePedestrianHypotheses(
  const vision_msgs::msg::Detection3D & detection)
{
  std::vector<TrajectoryHypothesis> hypotheses;

  rclcpp::Time current_time = node_->get_clock()->now();
  std::string frame_id = "map";

  TrajectoryHypothesis walk_hyp;
  walk_hyp.header.stamp = current_time;
  walk_hyp.header.frame_id = frame_id;
  walk_hyp.intent = Intent::CONTINUE_STRAIGHT;
  walk_hyp.probability = 0.0;

  double current_x = detection.bbox.center.position.x;
  double current_y = detection.bbox.center.position.y;
  double current_z = detection.bbox.center.position.z;
  double heading = extractYaw(detection.bbox.center.orientation);

  // Estimate speed from bounding box size heuristic
  double length = detection.bbox.size.x;
  double velocity = (length > 3.5) ? 5.0 : 1.4;

  for (double t = time_step_; t <= prediction_horizon_; t += time_step_) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = current_time + rclcpp::Duration::from_seconds(t);
    pose_stamped.pose.position.x = current_x + velocity * std::cos(heading) * t;
    pose_stamped.pose.position.y = current_y + velocity * std::sin(heading) * t;
    pose_stamped.pose.position.z = current_z;
    pose_stamped.pose.orientation = detection.bbox.center.orientation;

    walk_hyp.poses.push_back(pose_stamped);
  }

  hypotheses.push_back(walk_hyp);

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Pedestrian prediction: constant velocity");

  return hypotheses;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateCyclistHypotheses(
  const vision_msgs::msg::Detection3D & detection)
{
  std::vector<TrajectoryHypothesis> hypotheses;

  rclcpp::Time current_time = node_->get_clock()->now();
  std::string frame_id = "map";

  TrajectoryHypothesis cycle_hyp;
  cycle_hyp.header.stamp = current_time;
  cycle_hyp.header.frame_id = frame_id;
  cycle_hyp.intent = Intent::CONTINUE_STRAIGHT;
  cycle_hyp.probability = 0.0;

  double current_x = detection.bbox.center.position.x;
  double current_y = detection.bbox.center.position.y;
  double current_z = detection.bbox.center.position.z;
  double heading = extractYaw(detection.bbox.center.orientation);

  // Estimate speed from bounding box size heuristic
  double length = detection.bbox.size.x;
  double velocity = (length > 3.5) ? 5.0 : 1.4;

  for (double t = time_step_; t <= prediction_horizon_; t += time_step_) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = current_time + rclcpp::Duration::from_seconds(t);
    pose_stamped.pose.position.x = current_x + velocity * std::cos(heading) * t;
    pose_stamped.pose.position.y = current_y + velocity * std::sin(heading) * t;
    pose_stamped.pose.position.z = current_z;
    pose_stamped.pose.orientation = detection.bbox.center.orientation;

    cycle_hyp.poses.push_back(pose_stamped);
  }

  hypotheses.push_back(cycle_hyp);

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Cyclist prediction: constant velocity");

  return hypotheses;
}

}  // namespace prediction
