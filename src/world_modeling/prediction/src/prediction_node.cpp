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
    "/perception/detections_3D_tracked",
    10,
    std::bind(&PredictionNode::trackedObjectsCallback, this, std::placeholders::_1));

  ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose",
    10,
    std::bind(&PredictionNode::egoPoseCallback, this, std::placeholders::_1));

  // Initialize publishers
  // TODO: Uncomment when wato_msgs::msg::PredictionHypothesesArray is available
  // predictions_pub_ = this->create_publisher<wato_msgs::msg::PredictionHypothesesArray>(
  //   "/world_modeling/prediction/predicted_paths", 10);

  // Initialize core components
  trajectory_predictor_ = std::make_unique<TrajectoryPredictor>(
    this, prediction_horizon_, prediction_time_step_);
  
  intent_classifier_ = std::make_unique<IntentClassifier>(this);
  
  map_interface_ = std::make_unique<MapInterface>(this);

  RCLCPP_INFO(this->get_logger(), "Prediction node initialized successfully");
}

void PredictionNode::trackedObjectsCallback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  if (!ego_pose_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Ego pose not received yet, skipping prediction");
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Processing %zu tracked objects", msg->detections.size());

  // TODO: Create output message
  // wato_msgs::msg::PredictionHypothesesArray predictions_msg;
  // predictions_msg.header = msg->header;

  // Process each tracked object
  for (const auto & detection : msg->detections) {
    processObject(detection);
    
    // TODO: Add predictions to output message
    // predictions_msg.predictions.push_back(prediction);
  }

  // TODO: Publish predictions
  // predictions_pub_->publish(predictions_msg);
}

void PredictionNode::egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ego_pose_ = msg;
}

void PredictionNode::processObject(const vision_msgs::msg::Detection3D & detection)
{
  // PLACEHOLDER: Basic prediction pipeline with placeholders
  // TODO: Complete implementation with proper tracking integration
  
  RCLCPP_DEBUG(this->get_logger(), "Processing object with ID: %s", detection.id.c_str());
  
  try {
    // Step 1: Find current lanelet (using placeholder)
    geometry_msgs::msg::Point center = detection.bbox.center.position;
    int64_t current_lanelet = map_interface_->findNearestLanelet(center);
    
    // Step 2: Get possible future lanelets (using placeholder)
    auto future_lanelets = map_interface_->getPossibleFutureLanelets(
      current_lanelet, 3);
    
    // Step 3: Generate trajectory hypotheses
    auto hypotheses = trajectory_predictor_->generateHypotheses(
      detection, future_lanelets);
    
    if (hypotheses.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No hypotheses generated for object %s", 
                   detection.id.c_str());
      return;
    }
    
    // Step 4: Extract features for intent classification
    auto features = intent_classifier_->extractFeatures(detection, future_lanelets);
    
    // Step 5: Assign probabilities to hypotheses
    intent_classifier_->assignProbabilities(detection, hypotheses, features);
    
    // Step 6: Log results (would publish in real implementation)
    RCLCPP_DEBUG(this->get_logger(), 
                 "Generated %zu predictions for object %s",
                 hypotheses.size(), detection.id.c_str());
    
    // TODO: Create and publish wato_msgs::msg::PredictionHypotheses message
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), 
                 "Error processing object %s: %s", 
                 detection.id.c_str(), e.what());
  }
}

}  // namespace prediction

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<prediction::PredictionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
