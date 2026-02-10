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

#ifndef SIMPLE_PREDICTION__SIMPLE_PREDICTION_NODE_HPP_
#define SIMPLE_PREDICTION__SIMPLE_PREDICTION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model_msgs/msg/prediction.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace simple_prediction
{

/**
 * @brief Lifecycle node that bridges Detection3DArray to WorldObjectArray
 * with constant-velocity trajectory propagation.
 */
class SimplePredictionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SimplePredictionNode(const rclcpp::NodeOptions & options);
  ~SimplePredictionNode() override = default;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// Read parameters and create publisher.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /// Create subscription and activate publisher.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /// Reset subscription and deactivate publisher.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /// Release all resources.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /// Release all resources (terminal state).
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief Callback for incoming Detection3DArray messages from the tracker.
   *
   * Converts each detection to a WorldObject, generates constant-velocity
   * predictions, and publishes the resulting WorldObjectArray.
   *
   * @param msg Array of tracked 3D detections.
   */
  void trackedObjectsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  /**
   * @brief Generate constant-velocity predictions for a single detection.
   *
   * Extracts position and heading from the detection, estimates speed from
   * bounding-box length, and propagates a straight-line trajectory at fixed
   * time steps up to the prediction horizon.
   *
   * @param detection 3D detection with pose and bounding box.
   * @param frame_id Coordinate frame for the predicted poses.
   * @return Single-hypothesis prediction trajectory.
   */
  std::vector<world_model_msgs::msg::Prediction> generatePredictions(
    const vision_msgs::msg::Detection3D & detection, const std::string & frame_id);

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr tracked_objects_sub_;

  // Publisher
  rclcpp_lifecycle::LifecyclePublisher<world_model_msgs::msg::WorldObjectArray>::SharedPtr world_objects_pub_;

  // Parameters
  double prediction_horizon_;  // seconds
  double prediction_time_step_;  // seconds
  int64_t hypothesis_idx_;  // which results[] entry holds the entity class
};

}  // namespace simple_prediction

#endif  // SIMPLE_PREDICTION__SIMPLE_PREDICTION_NODE_HPP_
