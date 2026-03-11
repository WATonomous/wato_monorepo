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
 * @brief Trajectory generation for seed WorldObjects.
 *
 * Generates trajectory predictions for tracked objects to populate seed
 * WorldObjects. Uses motion models to propagate object state forward in time.
 */

#ifndef PREDICTION__TRAJECTORY_PREDICTOR_HPP_
#define PREDICTION__TRAJECTORY_PREDICTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "prediction/motion_models.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection3_d.hpp"

namespace prediction
{

/**
 * @brief Object types for prediction
 */
enum class ObjectType
{
  VEHICLE,
  PEDESTRIAN,
  CYCLIST,
  UNKNOWN
};

/**
 * @brief Intent types for trajectory hypotheses
 */
enum class Intent
{
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
struct TrajectoryHypothesis
{
  std_msgs::msg::Header header;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  Intent intent;
  double probability;
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
  TrajectoryPredictor(rclcpp_lifecycle::LifecycleNode * node, double prediction_horizon, double time_step);

  /**
   * @brief Generate trajectory hypotheses for a tracked object
   * @param detection Tracked object detection
   * @return Vector of trajectory hypotheses
   */
  std::vector<TrajectoryHypothesis> generateHypotheses(const vision_msgs::msg::Detection3D & detection);

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
  std::vector<TrajectoryHypothesis> generateVehicleHypotheses(const vision_msgs::msg::Detection3D & detection);

  /**
   * @brief Generate hypotheses for pedestrian objects
   */
  std::vector<TrajectoryHypothesis> generatePedestrianHypotheses(const vision_msgs::msg::Detection3D & detection);

  /**
   * @brief Generate hypotheses for cyclist objects
   */
  std::vector<TrajectoryHypothesis> generateCyclistHypotheses(const vision_msgs::msg::Detection3D & detection);

  rclcpp_lifecycle::LifecycleNode * node_;
  double prediction_horizon_;
  double time_step_;

  // Motion models
  std::unique_ptr<BicycleModel> bicycle_model_;
  std::unique_ptr<ConstantVelocityModel> constant_velocity_model_;
};

}  // namespace prediction

#endif  // PREDICTION__TRAJECTORY_PREDICTOR_HPP_
