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
 * @file prediction_node.hpp
 * @brief MAIN ORCHESTRATOR NODE - Coordinates the entire prediction pipeline
 * 
 * WHAT THIS FILE DOES:
 * - Subscribes to tracked objects from perception
 * - Subscribes to ego vehicle pose from localization  
 * - For each tracked object, orchestrates the prediction pipeline:
 *   1. Query map for current/future lanelets
 *   2. Call trajectory predictor to generate hypotheses
 *   3. Call intent classifier to assign probabilities
 *   4. Publish multi-modal predictions
 * 
 * WHO WORKS ON THIS:
 * - Integration work (all team members coordinate here)
 * - No person-specific tasks, this just calls the other modules
 * 
 * WHEN TO MODIFY:
 * - When adding new message publishers
 * - When changing the pipeline flow
 * - When integrating new predictor types
 */

#ifndef PREDICTION__PREDICTION_NODE_HPP_
#define PREDICTION__PREDICTION_NODE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "prediction/trajectory_predictor.hpp"
#include "prediction/intent_classifier.hpp"
#include "prediction/map_interface.hpp"

namespace prediction
{

/**
 * @brief Main prediction node that coordinates the multi-modal trajectory prediction pipeline
 * 
 * This node subscribes to tracked objects and ego pose, queries the HD map,
 * generates trajectory hypotheses, classifies intents, and publishes multi-modal predictions.
 */
class PredictionNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Prediction Node
   */
  explicit PredictionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Prediction Node
   */
  ~PredictionNode() override = default;

private:
  /**
   * @brief Callback for tracked objects
   * @param msg Tracked detections from perception
   */
  void trackedObjectsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  /**
   * @brief Callback for ego pose
   * @param msg Ego vehicle pose from localization
   */
  void egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Process a single tracked object and generate predictions
   * @param detection The tracked object to predict
   * @return TODO: PredictionHypotheses Multi-modal trajectory predictions
   */
  // TODO: Replace return type with wato_msgs::msg::PredictionHypotheses
  void processObject(const vision_msgs::msg::Detection3D & detection);

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr tracked_objects_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_sub_;

  // Publishers
  // TODO: Add publisher for wato_msgs::msg::PredictionHypothesesArray
  // rclcpp::Publisher<wato_msgs::msg::PredictionHypothesesArray>::SharedPtr predictions_pub_;

  // Core components
  std::unique_ptr<TrajectoryPredictor> trajectory_predictor_;
  std::unique_ptr<IntentClassifier> intent_classifier_;
  std::unique_ptr<MapInterface> map_interface_;

  // State
  geometry_msgs::msg::PoseStamped::SharedPtr ego_pose_;

  // Parameters
  double prediction_horizon_;      // seconds
  double prediction_time_step_;    // seconds
  bool use_map_constraints_;
  bool enable_visualization_;
};

}  // namespace prediction

#endif  // PREDICTION__PREDICTION_NODE_HPP_
