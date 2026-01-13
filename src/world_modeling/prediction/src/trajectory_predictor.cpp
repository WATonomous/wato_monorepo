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

namespace prediction
{

TrajectoryPredictor::TrajectoryPredictor(
  rclcpp::Node* node,
  double prediction_horizon,
  double time_step)
: node_(node),
  prediction_horizon_(prediction_horizon),
  time_step_(time_step)
{
  // Initialize motion models
  bicycle_model_ = std::make_unique<BicycleModel>();
  constant_velocity_model_ = std::make_unique<ConstantVelocityModel>();

  RCLCPP_INFO(node_->get_logger(), "TrajectoryPredictor initialized");
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateHypotheses(
  const vision_msgs::msg::Detection3D & detection,
  const std::vector<int64_t> & possible_lanelets)
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

ObjectType TrajectoryPredictor::classifyObjectType(
  const vision_msgs::msg::Detection3D & detection)
{
  // PLACEHOLDER: Simple rule-based classification
  // TODO: Implement proper classification using detection class labels and features
  
  RCLCPP_DEBUG(node_->get_logger(), "Classifying object type for detection: %s", 
               detection.id.c_str());
  
  // Use bounding box size as a simple heuristic
  double length = detection.bbox.size.x;
  double height = detection.bbox.size.z;
  
  // Vehicle: large length (> 3m)
  if (length > 3.5) {
    return ObjectType::VEHICLE;
  }
  // Pedestrian: small and tall
  else if (length < 1.0 && height > 1.0) {
    return ObjectType::PEDESTRIAN;
  }
  // Cyclist: medium size
  else if (length >= 1.0 && length <= 3.5) {
    return ObjectType::CYCLIST;
  }
  
  // Default to vehicle
  return ObjectType::VEHICLE;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateVehicleHypotheses(
  const vision_msgs::msg::Detection3D & detection,
  const std::vector<int64_t> & possible_lanelets)
{
  // ============================================================================
  // JOHN TASK: VEHICLE PREDICTION (BICYCLE KINEMATICS)
  // ============================================================================
  // TODO: Implement vehicle trajectory generation using bicycle model
  // - Query map_interface_ for lanelets and centerlines
  // - Use bicycle_model_->generateTrajectory() for path following
  // - Generate hypotheses for different intents (straight, turns, lane changes)
  // - Also implement BicycleModel::generateTrajectory() in motion_models.cpp
  // - Return std::vector<TrajectoryHypothesis> (see trajectory_predictor.hpp)
  // ============================================================================
  
  // PLACEHOLDER: Generate simple straight-line trajectory
  // TODO: Replace this with your full implementation
  std::vector<TrajectoryHypothesis> hypotheses;
  
  if (possible_lanelets.empty()) {
    return hypotheses;
  }

  // Create a simple "continue straight" hypothesis
  TrajectoryHypothesis straight_hyp;
  straight_hyp.intent = Intent::CONTINUE_STRAIGHT;
  straight_hyp.probability = 0.0;  // Will be set by classifier

  // Generate waypoints using constant velocity assumption
  double current_x = detection.bbox.center.position.x;
  double current_y = detection.bbox.center.position.y;
  double velocity = 5.0;  // Assume 5 m/s for now
  double heading = 0.0;   // Assume heading forward for now

  for (double t = 0.0; t <= prediction_horizon_; t += time_step_) {
    geometry_msgs::msg::Pose waypoint;
    waypoint.position.x = current_x + velocity * std::cos(heading) * t;
    waypoint.position.y = current_y + velocity * std::sin(heading) * t;
    waypoint.position.z = 0.0;
    waypoint.orientation.w = 1.0;
    
    straight_hyp.waypoints.push_back(waypoint);
    straight_hyp.timestamps.push_back(t);
  }

  hypotheses.push_back(straight_hyp);
  
  RCLCPP_DEBUG_ONCE(node_->get_logger(), 
                    "Using placeholder vehicle trajectories (simple straight-line)");
  
  return hypotheses;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generatePedestrianHypotheses(
  const vision_msgs::msg::Detection3D & detection,
  const std::vector<int64_t> & possible_lanelets)
{
  // ============================================================================
  // GIRISH TASK: PEDESTRIAN PREDICTION
  // ============================================================================
  // TODO: Implement pedestrian trajectory generation with constant velocity model
  // - Use constant_velocity_model_->generateTrajectory() with noise
  // - Check map_interface_->isCrosswalkNearby() for goal-directed behavior
  // - Return std::vector<TrajectoryHypothesis> (see trajectory_predictor.hpp)
  // ============================================================================
  
  // PLACEHOLDER: Generate simple constant velocity trajectory
  // TODO: Replace this with your full implementation
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
  
  RCLCPP_DEBUG_ONCE(node_->get_logger(), 
                    "Using placeholder pedestrian trajectories (simple constant velocity)");
  
  return hypotheses;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateCyclistHypotheses(
  const vision_msgs::msg::Detection3D & detection,
  const std::vector<int64_t> & possible_lanelets)
{
  // ============================================================================
  // ARUHANT TASK: CYCLIST PREDICTION (HYBRID MODEL)
  // ============================================================================
  // TODO: Implement hybrid cyclist model (pedestrian-like + vehicle-like)
  // - Research cyclist behavior at crosswalks vs on roads
  // - Use map_interface_->isCrosswalkNearby() to determine context
  // - Near crosswalk: use constant_velocity_model_ (pedestrian-like)
  // - On road: use bicycle_model_ (vehicle-like)
  // - IMPORTANT: Coordinate with Girish & John to match output format
  // - Return std::vector<TrajectoryHypothesis> (see trajectory_predictor.hpp)
  // ============================================================================
  
  // PLACEHOLDER: Use medium speed constant velocity
  // TODO: Replace this with your full hybrid implementation
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
  
  RCLCPP_DEBUG_ONCE(node_->get_logger(), 
                    "Using placeholder cyclist trajectories (simple constant velocity)");
  
  return hypotheses;
}

}  // namespace prediction
