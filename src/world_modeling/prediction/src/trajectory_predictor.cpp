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

#include <cmath>
#include <memory>  // for std::make_unique
#include <vector>  // for std::vector

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

  // Create a simple "continue straight" hypothesis
  TrajectoryHypothesis straight_hyp;
  straight_hyp.intent = Intent::CONTINUE_STRAIGHT;
  straight_hyp.probability = 0.0;  // Will be set by classifier

  // Extract state from detection
  double current_x = detection.bbox.center.position.x;
  double current_y = detection.bbox.center.position.y;
  double current_z = detection.bbox.center.position.z;
  double heading = extractYaw(detection.bbox.center.orientation);
  double velocity = 5.0;  // Vehicle speed ~5 m/s

  for (double t = time_step_; t <= prediction_horizon_; t += time_step_) {
    geometry_msgs::msg::Pose waypoint;
    waypoint.position.x = current_x + velocity * std::cos(heading) * t;
    waypoint.position.y = current_y + velocity * std::sin(heading) * t;
    waypoint.position.z = current_z;
    waypoint.orientation = detection.bbox.center.orientation;

    straight_hyp.waypoints.push_back(waypoint);
    straight_hyp.timestamps.push_back(t);
  }

  hypotheses.push_back(straight_hyp);

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Vehicle prediction: constant velocity");

  return hypotheses;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generatePedestrianHypotheses(
  const vision_msgs::msg::Detection3D & detection)
{
  std::vector<TrajectoryHypothesis> hypotheses;

  TrajectoryHypothesis walk_hyp;
  walk_hyp.intent = Intent::CONTINUE_STRAIGHT;
  walk_hyp.probability = 0.0;

  double current_x = detection.bbox.center.position.x;
  double current_y = detection.bbox.center.position.y;
  double current_z = detection.bbox.center.position.z;
  double heading = extractYaw(detection.bbox.center.orientation);
  double velocity = 1.4;  // Typical walking speed ~1.4 m/s

  for (double t = time_step_; t <= prediction_horizon_; t += time_step_) {
    geometry_msgs::msg::Pose waypoint;
    waypoint.position.x = current_x + velocity * std::cos(heading) * t;
    waypoint.position.y = current_y + velocity * std::sin(heading) * t;
    waypoint.position.z = current_z;
    waypoint.orientation = detection.bbox.center.orientation;

    walk_hyp.waypoints.push_back(waypoint);
    walk_hyp.timestamps.push_back(t);
  }

  hypotheses.push_back(walk_hyp);

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Pedestrian prediction: constant velocity");

  return hypotheses;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateCyclistHypotheses(
  const vision_msgs::msg::Detection3D & detection)
{
  std::vector<TrajectoryHypothesis> hypotheses;

  TrajectoryHypothesis cycle_hyp;
  cycle_hyp.intent = Intent::CONTINUE_STRAIGHT;
  cycle_hyp.probability = 0.0;

  double current_x = detection.bbox.center.position.x;
  double current_y = detection.bbox.center.position.y;
  double current_z = detection.bbox.center.position.z;
  double heading = extractYaw(detection.bbox.center.orientation);
  double velocity = 4.5;  // Typical cycling speed ~4.5 m/s

  for (double t = time_step_; t <= prediction_horizon_; t += time_step_) {
    geometry_msgs::msg::Pose waypoint;
    waypoint.position.x = current_x + velocity * std::cos(heading) * t;
    waypoint.position.y = current_y + velocity * std::sin(heading) * t;
    waypoint.position.z = current_z;
    waypoint.orientation = detection.bbox.center.orientation;

    cycle_hyp.waypoints.push_back(waypoint);
    cycle_hyp.timestamps.push_back(t);
  }

  hypotheses.push_back(cycle_hyp);

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Cyclist prediction: constant velocity");

  return hypotheses;
}

}  // namespace prediction
