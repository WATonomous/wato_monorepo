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
 * @brief Node that generates seed WorldObjects from tracked objects.
 *
 * Subscribes to tracked objects from perception, generates trajectory
 * predictions, and publishes seed WorldObjects for the world model.
 */

#ifndef PREDICTION__PREDICTION_NODE_HPP_
#define PREDICTION__PREDICTION_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "prediction/intent_classifier.hpp"
#include "prediction/trajectory_predictor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace prediction
{

/**
 * @brief Generates seed WorldObjects from tracked objects with trajectory predictions.
 *
 * Subscribes to tracked objects, generates trajectory predictions, and publishes
 * seed WorldObjects for consumption by the world model.
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
   * @brief Process a single tracked object and generate a seed WorldObject
   * @param detection The tracked object to predict
   * @param frame_id Coordinate frame for the predictions
   * @return WorldObject with predictions, or nullopt if generation failed
   */
  std::optional<world_model_msgs::msg::WorldObject> processObject(
    const vision_msgs::msg::Detection3D & detection, const std::string & frame_id);

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr tracked_objects_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_sub_;

  // Publishers
  rclcpp::Publisher<world_model_msgs::msg::WorldObjectArray>::SharedPtr world_objects_pub_;

  // Core components
  std::unique_ptr<TrajectoryPredictor> trajectory_predictor_;
  std::unique_ptr<IntentClassifier> intent_classifier_;

  // State
  geometry_msgs::msg::PoseStamped::SharedPtr ego_pose_;

  // Parameters
  double prediction_horizon_;  // seconds
  double prediction_time_step_;  // seconds
};

}  // namespace prediction

#endif  // PREDICTION__PREDICTION_NODE_HPP_
