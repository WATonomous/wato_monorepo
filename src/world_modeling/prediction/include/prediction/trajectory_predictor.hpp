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
 * @file trajectory_predictor.hpp
 * @brief TRAJECTORY GENERATION - Creates multiple trajectory hypotheses for each object
 * 
 * WHAT THIS FILE DOES:
 * - Classifies objects as vehicle/pedestrian/cyclist
 * - Generates multiple trajectory hypotheses (different intents/paths)
 * - Uses motion models to create realistic trajectories
 * - Returns trajectories as sequences of waypoints with timestamps
 * 
 * ========== TASK ASSIGNMENTS ==========
 * 
 * PERSON 1 - PEDESTRIAN PREDICTION:
 * - Implement: generatePedestrianHypotheses() in trajectory_predictor.cpp
 * - Use: ConstantVelocityModel from motion_models.hpp
 * - Add: Goal-directed behavior toward crosswalks
 * - Add: Multiple samples with noise for uncertainty
 * - Output: Vector of TrajectoryHypothesis with waypoints + timestamps
 * 
 * PERSON 2 - VEHICLE PREDICTION (BICYCLE KINEMATICS):
 * - Implement: generateVehicleHypotheses() in trajectory_predictor.cpp  
 * - Implement: BicycleModel path following in motion_models.cpp
 * - Generate hypotheses for: straight, left turn, right turn, lane changes
 * - Constrain trajectories to follow lanelet centerlines
 * - Output: Vector of TrajectoryHypothesis with waypoints + timestamps
 * 
 * PERSON 3 - CYCLIST PREDICTION (HYBRID MODEL):
 * - Implement: generateCyclistHypotheses() in trajectory_predictor.cpp
 * - Research: Cyclist behavior at crosswalks vs roads
 * - Use: Pedestrian model near crosswalks, vehicle model otherwise
 * - Coordinate: Ensure output format matches Person 1 & 2
 * - Output: Vector of TrajectoryHypothesis with waypoints + timestamps
 * 
 * ========== COORDINATION NOTES ==========
 * All three functions MUST return: std::vector<TrajectoryHypothesis>
 * 
 * TrajectoryHypothesis structure (defined below):
 * - waypoints: std::vector<geometry_msgs::msg::Pose>  (x,y,z positions)
 * - timestamps: std::vector<double>  (time for each waypoint)
 * - intent: Intent enum (CONTINUE_STRAIGHT, TURN_LEFT, etc.)
 * - probability: double (will be set by IntentClassifier, start at 0.0)
 * 
 * IMPORTANT: All team members use the same:
 * - prediction_horizon_ (from config, default 5.0 seconds)
 * - time_step_ (from config, default 0.1 seconds)
 * - Same coordinate frame (ego vehicle frame or global frame - TBD)
 */

#ifndef PREDICTION__TRAJECTORY_PREDICTOR_HPP_
#define PREDICTION__TRAJECTORY_PREDICTOR_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "prediction/motion_models.hpp"

namespace prediction
{

/**
 * @brief Object types for prediction
 */
enum class ObjectType {
  VEHICLE,
  PEDESTRIAN,
  CYCLIST,
  UNKNOWN
};

/**
 * @brief Intent types for trajectory hypotheses
 */
enum class Intent {
  CONTINUE_STRAIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  LANE_CHANGE_LEFT,
  LANE_CHANGE_RIGHT,
  STOP,
  UNKNOWN
};

/**
 * @brief Single trajectory hypothesis (sequence of waypoints)
 */
struct TrajectoryHypothesis {
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::vector<double> timestamps;
  Intent intent;
  double probability;  // Will be set by IntentClassifier
};

/**
 * @brief Generates trajectory hypotheses for tracked objects
 * 
 * Uses physics-based motion models to generate multiple trajectory hypotheses
 * representing different possible intents for each object type.
 */
class TrajectoryPredictor
{
public:
  /**
   * @brief Construct a new Trajectory Predictor
   * @param node ROS node pointer for logging
   * @param prediction_horizon Time horizon for predictions (seconds)
   * @param time_step Time step between waypoints (seconds)
   */
  TrajectoryPredictor(
    rclcpp::Node* node,
    double prediction_horizon,
    double time_step);

  /**
   * @brief Generate trajectory hypotheses for a tracked object
   * @param detection Tracked object detection
   * @param lanelet_info Information about current and possible future lanelets
   * @return Vector of trajectory hypotheses
   */
  std::vector<TrajectoryHypothesis> generateHypotheses(
    const vision_msgs::msg::Detection3D & detection,
    const std::vector<int64_t> & possible_lanelets);

  /**
   * @brief Classify object type based on detection information
   * @param detection Tracked object detection
   * @return ObjectType classification
   */
  ObjectType classifyObjectType(const vision_msgs::msg::Detection3D & detection);

private:
  /**
   * @brief Generate hypotheses for vehicle objects
   */
  std::vector<TrajectoryHypothesis> generateVehicleHypotheses(
    const vision_msgs::msg::Detection3D & detection,
    const std::vector<int64_t> & possible_lanelets);

  /**
   * @brief Generate hypotheses for pedestrian objects
   */
  std::vector<TrajectoryHypothesis> generatePedestrianHypotheses(
    const vision_msgs::msg::Detection3D & detection,
    const std::vector<int64_t> & possible_lanelets);

  /**
   * @brief Generate hypotheses for cyclist objects
   */
  std::vector<TrajectoryHypothesis> generateCyclistHypotheses(
    const vision_msgs::msg::Detection3D & detection,
    const std::vector<int64_t> & possible_lanelets);

  rclcpp::Node* node_;
  double prediction_horizon_;
  double time_step_;

  // Motion models
  std::unique_ptr<BicycleModel> bicycle_model_;
  std::unique_ptr<ConstantVelocityModel> constant_velocity_model_;
};

}  // namespace prediction

#endif  // PREDICTION__TRAJECTORY_PREDICTOR_HPP_
