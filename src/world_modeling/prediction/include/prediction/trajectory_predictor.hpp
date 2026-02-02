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

#include <lanelet2_core/primitives/Lanelet.h>

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "prediction/motion_models.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "world_model/lanelet_handler.hpp"

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
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::vector<double> timestamps;
  Intent intent;
  double probability;  // Will be set by IntentClassifier
};

/**
 * @brief Timestamped position for detection history tracking
 */
struct TimestampedPosition
{
  geometry_msgs::msg::Point position;
  double timestamp;  // Seconds since epoch
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
   * @brief Parameters for cyclist prediction (loaded from params.yaml)
   */
  struct CyclistParams
  {
    double lanelet_proximity_threshold;  // meters
    double min_speed;  // m/s
    double max_speed;  // m/s
    int max_lanelet_search_depth;  // max successive lanelets to follow
  };

  /**
   * @brief Construct a new Trajectory Predictor
   * @param node ROS node pointer for logging
   * @param prediction_horizon Time horizon for predictions (seconds)
   * @param time_step Time step between waypoints (seconds)
   * @param cyclist_params Cyclist-specific parameters (from params.yaml)
   * @param lanelet_handler Optional LaneletHandler for map queries
   */
  TrajectoryPredictor(
    rclcpp_lifecycle::LifecycleNode * node,
    double prediction_horizon,
    double time_step,
    const CyclistParams & cyclist_params,
    std::shared_ptr<world_model::LaneletHandler> lanelet_handler = nullptr);

  /**
   * @brief Set the LaneletHandler for map queries
   * @param lanelet_handler Shared pointer to LaneletHandler
   */
  void setLaneletHandler(std::shared_ptr<world_model::LaneletHandler> lanelet_handler);

  /**
   * @brief Generate trajectory hypotheses for a tracked object
   * @param detection Tracked object detection
   * @param timestamp Current timestamp in seconds
   * @return Vector of trajectory hypotheses
   */
  std::vector<TrajectoryHypothesis> generateHypotheses(
    const vision_msgs::msg::Detection3D & detection, double timestamp);

  /**
   * @brief Classify object type based on detection information
   * @param detection Tracked object detection
   * @return ObjectType classification
   */
  ObjectType classifyObjectType(const vision_msgs::msg::Detection3D & detection) const;

private:
  /**
   * @brief Generate hypotheses for vehicle objects
   * @param velocity Computed velocity, or nullopt if insufficient history
   */
  std::vector<TrajectoryHypothesis> generateVehicleHypotheses(
    const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity);

  /**
   * @brief Generate hypotheses for pedestrian objects
   * @param velocity Computed velocity, or nullopt if insufficient history
   */
  std::vector<TrajectoryHypothesis> generatePedestrianHypotheses(
    const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity);

  /**
   * @brief Generate hypotheses for cyclist objects (hybrid model)
   * @param velocity Computed velocity, or nullopt if insufficient history
   */
  std::vector<TrajectoryHypothesis> generateCyclistHypotheses(
    const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity);

  /**
   * @brief Update detection history and compute velocity
   * @param object_id Unique object identifier
   * @param position Current position
   * @param timestamp Current timestamp
   * @return Computed velocity magnitude (m/s), or nullopt if insufficient
   * history
   */
  std::optional<double> updateHistoryAndComputeVelocity(
    const std::string & object_id, const geometry_msgs::msg::Point & position, double timestamp);

  /**
   * @brief Get all possible lanelet paths (for multi-hypothesis generation)
   * @param start_lanelet Starting lanelet
   * @param max_depth Maximum number of successive lanelets to follow
   * @return Vector of (path_points, intent) pairs for each possible route
   */
  std::vector<std::pair<std::vector<Eigen::Vector2d>, Intent>> getAllLaneletPaths(
    const lanelet::ConstLanelet & start_lanelet, int max_depth) const;

  rclcpp_lifecycle::LifecycleNode * node_;
  double prediction_horizon_;
  double time_step_;

  // Motion models
  std::unique_ptr<BicycleModel> bicycle_model_;
  std::unique_ptr<ConstantVelocityModel> constant_velocity_model_;

  // LaneletHandler for map queries (optional, protected by mutex for thread
  // safety)
  mutable std::shared_mutex lanelet_handler_mutex_;
  std::shared_ptr<world_model::LaneletHandler> lanelet_handler_;

  // Cyclist-specific parameters (const after construction)
  const CyclistParams cyclist_params_;

  // Detection history for velocity computation (object_id -> history)
  // Protected by mutex for thread-safe access from multi-threaded executor
  mutable std::mutex history_mutex_;
  std::unordered_map<std::string, std::deque<TimestampedPosition>> detection_history_;

  // History parameters
  static constexpr double kHistoryMaxAge = 2.0;  // seconds
  static constexpr size_t kHistoryMaxSize = 10;  // entries per object
};

}  // namespace prediction

#endif  // PREDICTION__TRAJECTORY_PREDICTOR_HPP_
