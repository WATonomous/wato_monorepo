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

#include "prediction/map_interface.hpp"  // for HD map queries
#include "rclcpp/rclcpp.hpp"  // Add this for rclcpp::Duration

namespace prediction
{

TrajectoryPredictor::TrajectoryPredictor(rclcpp::Node * node, double prediction_horizon, double time_step)
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
  const vision_msgs::msg::Detection3D & detection, const std::vector<int64_t> & possible_lanelets)
{
  ObjectType obj_type = classifyObjectType(detection);

  std::vector<TrajectoryHypothesis> hypotheses;

  switch (obj_type) {
    case ObjectType::VEHICLE:
      hypotheses = generateVehicleHypotheses(detection, possible_lanelets);
      break;
    case ObjectType::PEDESTRIAN:
      hypotheses = generatePedestrianHypotheses(detection, possible_lanelets);
      break;
    case ObjectType::CYCLIST:
      hypotheses = generateCyclistHypotheses(detection, possible_lanelets);
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
  const vision_msgs::msg::Detection3D & detection, const std::vector<int64_t> & possible_lanelets)
{
  std::vector<TrajectoryHypothesis> hypotheses;

  if (possible_lanelets.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No possible lanelets provided, cannot generate vehicle hypotheses");
    return hypotheses;
  }

  // Get current time and frame for timestamps
  rclcpp::Time current_time = node_->get_clock()->now();
  std::string frame_id = "map";

  // Extract initial state, velocity and heading from detection
  KinematicState initial_state;
  initial_state.x = detection.bbox.center.position.x;
  initial_state.y = detection.bbox.center.position.y;

  if (!detection.results.empty() && detection.results[0].hypothesis.score > 0.0) {
    initial_state.v = detection.results[0].hypothesis.score;
  } else {
    initial_state.v = 5.0;
  }

  const auto & heading = detection.bbox.center.orientation;
  initial_state.theta = std::atan2(
    2.0 * (heading.w * heading.z + heading.x * heading.y), 1.0 - 2.0 * (heading.y * heading.y + heading.z * heading.z));

  initial_state.a = 0.0;
  initial_state.delta = 0.0;

  MapInterface map_interface(node_);

  // HYPOTHESES GENERATION
  // ==== Continue Straight ====
  if (!possible_lanelets.empty()) {
    TrajectoryHypothesis straight_hyp;
    straight_hyp.header.stamp = current_time;
    straight_hyp.header.frame_id = frame_id;
    straight_hyp.intent = Intent::CONTINUE_STRAIGHT;
    straight_hyp.probability = 0.0;  // Will be set by classifier

    // Get centerline of the first possible lanelet
    LaneletInfo current_lane = map_interface.getLaneletById(possible_lanelets[0]);

    // Convert centerline to Eigen format for bicycle model
    std::vector<Eigen::Vector2d> straight_path;
    for (const auto & point : current_lane.centerline) {
      straight_path.push_back(Eigen::Vector2d(point.x, point.y));
    }

    if (!straight_path.empty()) {
      straight_hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, straight_path, prediction_horizon_, time_step_, current_time, frame_id);
      hypotheses.push_back(straight_hyp);
    }
  }

  // ==== Turns (LEFT/RIGHT) ====
  auto future_lanelets = map_interface.getPossibleFutureLanelets(possible_lanelets[0], 2);

  for (size_t i = 1; i < future_lanelets.size() && i <= 3; ++i) {
    LaneletInfo turn_lane = map_interface.getLaneletById(future_lanelets[i]);

    // Check if this is a turn by comparing heading
    if (!turn_lane.centerline.empty() && turn_lane.centerline.size() >= 2) {
      double lane_heading = std::atan2(
        turn_lane.centerline[1].y - turn_lane.centerline[0].y, turn_lane.centerline[1].x - turn_lane.centerline[0].x);

      double heading_diff = lane_heading - initial_state.theta;
      while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
      while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;

      // Determine turn type based on heading change
      Intent turn_intent;
      if (heading_diff > 0.26) {  // >15 degrees = left turn
        turn_intent = Intent::TURN_LEFT;
      } else if (heading_diff < -0.26) {  // <-15 degrees = right turn
        turn_intent = Intent::TURN_RIGHT;
      } else {
        continue;  // Angle not significant enough for a turn
      }

      TrajectoryHypothesis turn_hyp;
      turn_hyp.header.stamp = current_time;
      turn_hyp.header.frame_id = frame_id;
      turn_hyp.intent = turn_intent;
      turn_hyp.probability = 0.0;

      std::vector<Eigen::Vector2d> turn_path;
      for (const auto & point : turn_lane.centerline) {
        turn_path.push_back(Eigen::Vector2d(point.x, point.y));
      }

      turn_hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, turn_path, prediction_horizon_, time_step_, current_time, frame_id);
      hypotheses.push_back(turn_hyp);
    }
  }

  // ==== Lane Change Left ====
  if (possible_lanelets.size() > 1) {
    LaneletInfo left_lane = map_interface.getLaneletById(possible_lanelets[1]);

    if (!left_lane.centerline.empty()) {
      TrajectoryHypothesis left_change_hyp;
      left_change_hyp.header.stamp = current_time;
      left_change_hyp.header.frame_id = frame_id;
      left_change_hyp.intent = Intent::LANE_CHANGE_LEFT;
      left_change_hyp.probability = 0.0;

      std::vector<Eigen::Vector2d> left_change_path;
      for (const auto & point : left_lane.centerline) {
        left_change_path.push_back(Eigen::Vector2d(point.x, point.y));
      }

      left_change_hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, left_change_path, prediction_horizon_, time_step_, current_time, frame_id);
      hypotheses.push_back(left_change_hyp);
    }
  }

  // ==== Lane Change Right ====
  if (possible_lanelets.size() > 2) {
    LaneletInfo right_lane = map_interface.getLaneletById(possible_lanelets[2]);

    if (!right_lane.centerline.empty()) {
      TrajectoryHypothesis right_change_hyp;
      right_change_hyp.header.stamp = current_time;
      right_change_hyp.header.frame_id = frame_id;
      right_change_hyp.intent = Intent::LANE_CHANGE_RIGHT;
      right_change_hyp.probability = 0.0;

      std::vector<Eigen::Vector2d> right_change_path;
      for (const auto & point : right_lane.centerline) {
        right_change_path.push_back(Eigen::Vector2d(point.x, point.y));
      }

      right_change_hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, right_change_path, prediction_horizon_, time_step_, current_time, frame_id);
      hypotheses.push_back(right_change_hyp);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Generated %zu vehicle trajectory hypotheses", hypotheses.size());

  return hypotheses;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generatePedestrianHypotheses(
  const vision_msgs::msg::Detection3D & detection, const std::vector<int64_t> & possible_lanelets)
{
  std::vector<TrajectoryHypothesis> hypotheses;

  TrajectoryHypothesis walk_hyp;
  walk_hyp.intent = Intent::CONTINUE_STRAIGHT;
  walk_hyp.probability = 0.0;

  double current_x = detection.bbox.center.position.x;
  double current_y = detection.bbox.center.position.y;
  double velocity = 1.4;  // Typical walking speed ~1.4 m/s
  double heading = 0.0;

  for (double t = 0.0; t <= prediction_horizon_; t += time_step_) {
    geometry_msgs::msg::Pose waypoint;
    waypoint.position.x = current_x + velocity * std::cos(heading) * t;
    waypoint.position.y = current_y + velocity * std::sin(heading) * t;
    waypoint.position.z = 0.0;
    waypoint.orientation.w = 1.0;

    walk_hyp.waypoints.push_back(waypoint);
    walk_hyp.timestamps.push_back(t);
  }

  hypotheses.push_back(walk_hyp);

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Using placeholder pedestrian trajectories (simple constant velocity)");

  return hypotheses;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateCyclistHypotheses(
  const vision_msgs::msg::Detection3D & detection, const std::vector<int64_t> & possible_lanelets)
{
  std::vector<TrajectoryHypothesis> hypotheses;

  TrajectoryHypothesis cycle_hyp;
  cycle_hyp.intent = Intent::CONTINUE_STRAIGHT;
  cycle_hyp.probability = 0.0;

  double current_x = detection.bbox.center.position.x;
  double current_y = detection.bbox.center.position.y;
  double velocity = 4.5;  // Typical cycling speed ~4.5 m/s
  double heading = 0.0;

  for (double t = 0.0; t <= prediction_horizon_; t += time_step_) {
    geometry_msgs::msg::Pose waypoint;
    waypoint.position.x = current_x + velocity * std::cos(heading) * t;
    waypoint.position.y = current_y + velocity * std::sin(heading) * t;
    waypoint.position.z = 0.0;
    waypoint.orientation.w = 1.0;

    cycle_hyp.waypoints.push_back(waypoint);
    cycle_hyp.timestamps.push_back(t);
  }

  hypotheses.push_back(cycle_hyp);

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Using placeholder cyclist trajectories (simple constant velocity)");

  return hypotheses;
}

}  // namespace prediction
