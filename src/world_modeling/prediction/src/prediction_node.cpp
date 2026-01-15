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

#include "prediction/prediction_node.hpp"

#include <memory>  // for std::make_unique

namespace prediction
{

PredictionNode::PredictionNode(const rclcpp::NodeOptions & options)
: Node("prediction_node", options)
{
  // Declare parameters
  this->declare_parameter("prediction_horizon", 5.0);
  this->declare_parameter("prediction_time_step", 0.1);
  this->declare_parameter("use_map_constraints", true);
  this->declare_parameter("enable_visualization", false);

  // Get parameters
  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
  prediction_time_step_ = this->get_parameter("prediction_time_step").as_double();
  use_map_constraints_ = this->get_parameter("use_map_constraints").as_bool();
  enable_visualization_ = this->get_parameter("enable_visualization").as_bool();

  RCLCPP_INFO(this->get_logger(), "Prediction horizon: %.2f seconds", prediction_horizon_);
  RCLCPP_INFO(this->get_logger(), "Prediction time step: %.2f seconds", prediction_time_step_);

  // Initialize subscribers
  tracked_objects_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "tracks_3d", 10, std::bind(&PredictionNode::trackedObjectsCallback, this, std::placeholders::_1));

  ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "ego_pose", 10, std::bind(&PredictionNode::egoPoseCallback, this, std::placeholders::_1));

  trajectory_predictor_ = std::make_unique<TrajectoryPredictor>(this, prediction_horizon_, prediction_time_step_);

  intent_classifier_ = std::make_unique<IntentClassifier>(this);

  map_interface_ = std::make_unique<MapInterface>(this);
  RCLCPP_INFO(this->get_logger(), "Prediction node initialized successfully");
}

void PredictionNode::trackedObjectsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  if (!ego_pose_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Ego pose not received yet, skipping prediction");
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Processing %zu tracked objects", msg->detections.size());

  for (const auto & detection : msg->detections) {
    processObject(detection);
  }
}

void PredictionNode::egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ego_pose_ = msg;
}

void PredictionNode::processObject(const vision_msgs::msg::Detection3D & detection)
{
  RCLCPP_DEBUG(this->get_logger(), "Processing object with ID: %s", detection.id.c_str());

  try {
    geometry_msgs::msg::Point center = detection.bbox.center.position;
    int64_t current_lanelet = map_interface_->findNearestLanelet(center);

    auto future_lanelets = map_interface_->getPossibleFutureLanelets(current_lanelet, 3);

    auto hypotheses = trajectory_predictor_->generateHypotheses(detection, future_lanelets);

    if (hypotheses.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No hypotheses generated for object %s", detection.id.c_str());
      return;
    }

    auto features = intent_classifier_->extractFeatures(detection, future_lanelets);
    intent_classifier_->assignProbabilities(detection, hypotheses, features);

    RCLCPP_DEBUG(
      this->get_logger(), "Generated %zu predictions for object %s", hypotheses.size(), detection.id.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing object %s: %s", detection.id.c_str(), e.what());
  }
}

}  // namespace prediction

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prediction::PredictionNode)
