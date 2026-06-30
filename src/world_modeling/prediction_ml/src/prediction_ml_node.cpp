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

#include "prediction_ml/prediction_ml_node.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "prediction_ml/fallback_prediction.hpp"

namespace prediction_ml
{

PredictionMlNode::PredictionMlNode(const rclcpp::NodeOptions & options)
: LifecycleNode("prediction_ml_node", options)
{
  this->declare_parameter("prediction_horizon", 3.0);
  this->declare_parameter("prediction_time_step", 0.2);
  this->declare_parameter("fallback.vehicle_size_threshold_m", 3.5);
  this->declare_parameter("fallback.vehicle_speed_mps", 5.0);
  this->declare_parameter("fallback.vru_speed_mps", 1.4);
  this->declare_parameter("mtr.enabled", false);
  this->declare_parameter("mtr.request_topic", "/mtr/scenes");
  this->declare_parameter("mtr.result_topic", "/mtr/predictions");
  this->declare_parameter("mtr.cache_ttl_s", 0.5);
  RCLCPP_INFO(this->get_logger(), "PredictionMlNode created (unconfigured)");
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_configure(const rclcpp_lifecycle::State &)
{
  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
  prediction_time_step_ = this->get_parameter("prediction_time_step").as_double();
  fallback_vehicle_size_threshold_m_ = this->get_parameter("fallback.vehicle_size_threshold_m").as_double();
  fallback_vehicle_speed_mps_ = this->get_parameter("fallback.vehicle_speed_mps").as_double();
  fallback_vru_speed_mps_ = this->get_parameter("fallback.vru_speed_mps").as_double();
  mtr_enabled_ = this->get_parameter("mtr.enabled").as_bool();
  mtr_request_topic_ = this->get_parameter("mtr.request_topic").as_string();
  mtr_result_topic_ = this->get_parameter("mtr.result_topic").as_string();
  const double cache_ttl_s = this->get_parameter("mtr.cache_ttl_s").as_double();

  scene_adapter_ = std::make_unique<MtrSceneAdapter>();
  result_cache_ = std::make_unique<MtrResultCache>(cache_ttl_s);

  world_objects_pub_ = this->create_publisher<world_model_msgs::msg::WorldObjectArray>("world_object_seeds", 10);
  if (mtr_enabled_) {
    mtr_scene_pub_ = this->create_publisher<deep_msgs::msg::MtrScene>(mtr_request_topic_, 10);
    mtr_result_sub_ = this->create_subscription<deep_msgs::msg::MtrPredictionArray>(
      mtr_result_topic_, 10, std::bind(&PredictionMlNode::mtrResultCallback, this, std::placeholders::_1));
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Configured (horizon=%.1fs, step=%.2fs, mtr=%s)",
    prediction_horizon_,
    prediction_time_step_,
    mtr_enabled_ ? "enabled" : "disabled");
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_activate(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "tracks_3d", 10, std::bind(&PredictionMlNode::trackedObjectsCallback, this, std::placeholders::_1));
  ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "ego_pose", 10, std::bind(&PredictionMlNode::egoPoseCallback, this, std::placeholders::_1));
  lanelet_ahead_sub_ = this->create_subscription<lanelet_msgs::msg::LaneletAhead>(
    "lanelet_ahead", 10, std::bind(&PredictionMlNode::laneletAheadCallback, this, std::placeholders::_1));
  world_objects_pub_->on_activate();
  if (mtr_scene_pub_) {
    mtr_scene_pub_->on_activate();
  }
  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  world_objects_pub_->on_deactivate();
  if (mtr_scene_pub_) {
    mtr_scene_pub_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  mtr_result_sub_.reset();
  mtr_scene_pub_.reset();
  world_objects_pub_.reset();
  scene_adapter_.reset();
  result_cache_.reset();
  ego_pose_.reset();
  lanelet_ahead_.reset();
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  mtr_result_sub_.reset();
  if (mtr_scene_pub_) {
    mtr_scene_pub_->on_deactivate();
    mtr_scene_pub_.reset();
  }
  if (world_objects_pub_) {
    world_objects_pub_->on_deactivate();
    world_objects_pub_.reset();
  }
  scene_adapter_.reset();
  result_cache_.reset();
  ego_pose_.reset();
  lanelet_ahead_.reset();
  return CallbackReturn::SUCCESS;
}

void PredictionMlNode::egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ego_pose_ = msg;
}

void PredictionMlNode::laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg)
{
  lanelet_ahead_ = msg;
}

void PredictionMlNode::mtrResultCallback(const deep_msgs::msg::MtrPredictionArray::ConstSharedPtr msg)
{
  if (!result_cache_ || !result_cache_->accept(*msg, this->get_clock()->now().seconds())) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Rejected stale, unknown, or malformed MTR result");
  }
}

std::vector<world_model_msgs::msg::WorldObject> PredictionMlNode::buildFallback(
  const vision_msgs::msg::Detection3DArray & msg) const
{
  std::vector<world_model_msgs::msg::WorldObject> objects;
  const std::string & frame_id = msg.header.frame_id;
  for (const auto & detection : msg.detections) {
    world_model_msgs::msg::WorldObject obj;
    obj.detection = detection;

    const double x = detection.bbox.center.position.x;
    const double y = detection.bbox.center.position.y;
    const double z = detection.bbox.center.position.z;
    const auto & q = detection.bbox.center.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double speed = selectFallbackSpeed(
      detection.bbox.size.x, fallback_vehicle_size_threshold_m_, fallback_vehicle_speed_mps_, fallback_vru_speed_mps_);

    world_model_msgs::msg::Prediction pred;
    pred.header.frame_id = frame_id;
    pred.conf = 1.0;
    const int num_steps = static_cast<int>(std::round(prediction_horizon_ / prediction_time_step_));
    for (int i = 1; i <= num_steps; ++i) {
      const double t = i * prediction_time_step_;
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = frame_id;
      ps.pose.position.x = x + speed * std::cos(yaw) * t;
      ps.pose.position.y = y + speed * std::sin(yaw) * t;
      ps.pose.position.z = z;
      ps.pose.orientation = detection.bbox.center.orientation;
      pred.poses.push_back(ps);
    }
    obj.predictions.push_back(pred);
    objects.push_back(obj);
  }
  return objects;
}

void PredictionMlNode::trackedObjectsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  const double now_s = this->get_clock()->now().seconds();
  auto fallback = buildFallback(*msg);

  if (mtr_enabled_ && mtr_scene_pub_ && mtr_scene_pub_->is_activated()) {
    auto scene = scene_adapter_->build(*msg, ego_pose_.get(), lanelet_ahead_.get());
    result_cache_->rememberRequest(scene);
    mtr_scene_pub_->publish(scene);
  }

  world_model_msgs::msg::WorldObjectArray output;
  output.header = msg->header;
  output.objects = result_cache_->select(fallback, now_s);
  if (world_objects_pub_ && world_objects_pub_->is_activated()) {
    world_objects_pub_->publish(output);
  }
}

}  // namespace prediction_ml
