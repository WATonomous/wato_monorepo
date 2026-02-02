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

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace prediction
{

PredictionNode::PredictionNode(const rclcpp::NodeOptions & options)
: LifecycleNode("prediction_node", options)
{
  // Declare parameters (defaults match simple_prediction for drop-in compatibility)
  this->declare_parameter("prediction_horizon", 3.0);
  this->declare_parameter("prediction_time_step", 0.2);
  this->declare_parameter("max_lanelet_search_depth", 3);
  this->declare_parameter("cyclist_speed_range", std::vector<double>{2.0, 8.0});
  this->declare_parameter("lanelet_proximity_threshold", 5.0);

  RCLCPP_INFO(this->get_logger(), "PredictionNode created (unconfigured)");
}

PredictionNode::CallbackReturn PredictionNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  // Get parameters
  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
  prediction_time_step_ = this->get_parameter("prediction_time_step").as_double();

  // Build cyclist params from ROS params
  TrajectoryPredictor::CyclistParams cyclist_params;
  cyclist_params.max_lanelet_search_depth = this->get_parameter("max_lanelet_search_depth").as_int();
  cyclist_params.lanelet_proximity_threshold = this->get_parameter("lanelet_proximity_threshold").as_double();

  auto speed_range = this->get_parameter("cyclist_speed_range").as_double_array();
  if (speed_range.size() >= 2) {
    cyclist_params.min_speed = speed_range[0];
    cyclist_params.max_speed = speed_range[1];
  }

  RCLCPP_INFO(this->get_logger(), "Prediction horizon: %.2f seconds", prediction_horizon_);
  RCLCPP_INFO(this->get_logger(), "Prediction time step: %.2f seconds", prediction_time_step_);
  RCLCPP_INFO(
    this->get_logger(),
    "Cyclist params: speed=[%.1f, %.1f] m/s, lanelet_threshold=%.1f m, max_depth=%d",
    cyclist_params.min_speed,
    cyclist_params.max_speed,
    cyclist_params.lanelet_proximity_threshold,
    cyclist_params.max_lanelet_search_depth);

  // Initialize publisher
  world_objects_pub_ = this->create_publisher<world_model_msgs::msg::WorldObjectArray>("world_object_seeds", 10);

  // Initialize components
  trajectory_predictor_ =
    std::make_unique<TrajectoryPredictor>(this, prediction_horizon_, prediction_time_step_, cyclist_params);
  intent_classifier_ = std::make_unique<IntentClassifier>(this);

  RCLCPP_INFO(this->get_logger(), "Configured successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn PredictionNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  // Initialize subscribers
  tracked_objects_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "tracks_3d", 10, std::bind(&PredictionNode::trackedObjectsCallback, this, std::placeholders::_1));

  ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "ego_pose", 10, std::bind(&PredictionNode::egoPoseCallback, this, std::placeholders::_1));

  world_objects_pub_->on_activate();

  RCLCPP_INFO(this->get_logger(), "Activated successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn PredictionNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  world_objects_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivated successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn PredictionNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  // Reset subscribers and publishers
  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  world_objects_pub_.reset();

  // Reset components
  trajectory_predictor_.reset();
  intent_classifier_.reset();

  // Clear state
  ego_pose_.reset();

  RCLCPP_INFO(this->get_logger(), "Cleaned up successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn PredictionNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  // Reset subscribers and publishers
  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  world_objects_pub_.reset();

  // Reset components
  trajectory_predictor_.reset();
  intent_classifier_.reset();

  // Clear state
  ego_pose_.reset();

  return CallbackReturn::SUCCESS;
}

void PredictionNode::trackedObjectsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Processing %zu tracked objects", msg->detections.size());

  world_model_msgs::msg::WorldObjectArray output;
  output.header = msg->header;

  // Extract timestamp from header
  double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  for (const auto & detection : msg->detections) {
    auto world_obj = processObject(detection, msg->header.frame_id, timestamp);
    if (world_obj.has_value()) {
      output.objects.push_back(world_obj.value());
    }
  }

  world_objects_pub_->publish(output);
}

void PredictionNode::egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ego_pose_ = msg;
}

std::optional<world_model_msgs::msg::WorldObject> PredictionNode::processObject(
  const vision_msgs::msg::Detection3D & detection, const std::string & frame_id, double timestamp)
{
  RCLCPP_DEBUG(this->get_logger(), "Processing object with ID: %s", detection.id.c_str());

  try {
    auto hypotheses = trajectory_predictor_->generateHypotheses(detection, timestamp);

    if (hypotheses.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No hypotheses generated for object %s", detection.id.c_str());
      return std::nullopt;
    }

    auto features = intent_classifier_->extractFeatures(detection);
    intent_classifier_->assignProbabilities(detection, hypotheses, features);

    // Build WorldObject with predictions
    world_model_msgs::msg::WorldObject world_obj;
    world_obj.detection = detection;

    for (const auto & hypothesis : hypotheses) {
      world_model_msgs::msg::Prediction pred;
      pred.header.frame_id = frame_id;
      pred.conf = hypothesis.probability;

      for (size_t i = 0; i < hypothesis.waypoints.size(); ++i) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id;
        ps.pose = hypothesis.waypoints[i];
        pred.poses.push_back(ps);
      }

      world_obj.predictions.push_back(pred);
    }

    RCLCPP_DEBUG(
      this->get_logger(), "Generated %zu predictions for object %s", hypotheses.size(), detection.id.c_str());

    return world_obj;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing object %s: %s", detection.id.c_str(), e.what());
    return std::nullopt;
  }
}

}  // namespace prediction

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<prediction::PredictionNode>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
