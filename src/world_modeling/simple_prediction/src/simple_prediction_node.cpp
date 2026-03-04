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

#include "simple_prediction/simple_prediction_node.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace simple_prediction
{

SimplePredictionNode::SimplePredictionNode(const rclcpp::NodeOptions & options)
: LifecycleNode("simple_prediction_node", options)
{
  this->declare_parameter("prediction_horizon", 3.0);
  this->declare_parameter("prediction_time_step", 0.2);
  this->declare_parameter<int64_t>("entity_class_hypothesis_index", 0);

  RCLCPP_INFO(this->get_logger(), "SimplePredictionNode created (unconfigured)");
}

SimplePredictionNode::CallbackReturn SimplePredictionNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
  prediction_time_step_ = this->get_parameter("prediction_time_step").as_double();
  hypothesis_idx_ = this->get_parameter("entity_class_hypothesis_index").as_int();

  world_objects_pub_ = this->create_publisher<world_model_msgs::msg::WorldObjectArray>("world_object_seeds", 10);

  RCLCPP_INFO(this->get_logger(), "Configured (horizon=%.1fs, step=%.2fs)", prediction_horizon_, prediction_time_step_);
  return CallbackReturn::SUCCESS;
}

SimplePredictionNode::CallbackReturn SimplePredictionNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  tracked_objects_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "tracks_3d", 10, std::bind(&SimplePredictionNode::trackedObjectsCallback, this, std::placeholders::_1));

  world_objects_pub_->on_activate();

  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

SimplePredictionNode::CallbackReturn SimplePredictionNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  tracked_objects_sub_.reset();
  world_objects_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

SimplePredictionNode::CallbackReturn SimplePredictionNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  tracked_objects_sub_.reset();
  world_objects_pub_.reset();

  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

SimplePredictionNode::CallbackReturn SimplePredictionNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  tracked_objects_sub_.reset();
  world_objects_pub_.reset();

  return CallbackReturn::SUCCESS;
}

void SimplePredictionNode::trackedObjectsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  world_model_msgs::msg::WorldObjectArray output;
  output.header = msg->header;

  const std::string & frame_id = msg->header.frame_id;

  for (const auto & detection : msg->detections) {
    world_model_msgs::msg::WorldObject obj;
    obj.detection = detection;

    bool is_traffic_light = static_cast<size_t>(hypothesis_idx_) < detection.results.size() &&
                            detection.results[hypothesis_idx_].hypothesis.class_id == "traffic_light";
    if (!is_traffic_light) {
      obj.predictions = generatePredictions(detection, frame_id);
    }

    output.objects.push_back(obj);
  }

  world_objects_pub_->publish(output);
}

std::vector<world_model_msgs::msg::Prediction> SimplePredictionNode::generatePredictions(
  const vision_msgs::msg::Detection3D & detection, const std::string & frame_id)
{
  std::vector<world_model_msgs::msg::Prediction> predictions;

  // Extract current position from detection
  double x = detection.bbox.center.position.x;
  double y = detection.bbox.center.position.y;
  double z = detection.bbox.center.position.z;

  // Extract heading from quaternion (yaw only)
  const auto & q = detection.bbox.center.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Estimate speed from bounding box size heuristic:
  // large objects (vehicles) ~5 m/s, small objects (pedestrians) ~1.4 m/s
  double length = detection.bbox.size.x;
  double speed = (length > 3.5) ? 5.0 : 1.4;

  // Single constant-velocity hypothesis (continue straight)
  world_model_msgs::msg::Prediction pred;
  pred.header.frame_id = frame_id;
  pred.conf = 1.0;

  for (double t = prediction_time_step_; t <= prediction_horizon_; t += prediction_time_step_) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id;
    ps.pose.position.x = x + speed * std::cos(yaw) * t;
    ps.pose.position.y = y + speed * std::sin(yaw) * t;
    ps.pose.position.z = z;
    ps.pose.orientation = detection.bbox.center.orientation;
    pred.poses.push_back(ps);
  }

  predictions.push_back(pred);
  return predictions;
}

}  // namespace simple_prediction

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<simple_prediction::SimplePredictionNode>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
