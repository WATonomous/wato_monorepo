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
#include <utility>
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
  this->declare_parameter("mtr.mode", "disabled");
  this->declare_parameter("mtr.engine_path", "");
  this->declare_parameter("mtr.metadata_path", "");
  this->declare_parameter("mtr.cache_ttl_s", 0.5);
  this->declare_parameter("mtr.selected_target_agent_limit", 8);
  this->declare_parameter("mtr.target_forward_half_angle_deg", 90.0);
  this->declare_parameter("mtr.ego_length", 4.0936);
  this->declare_parameter("mtr.ego_width", 1.3579);
  this->declare_parameter("mtr.ego_height", 1.4090);
  this->declare_parameter("mtr.ego_velocity_child_frame", "base_footprint");
  this->declare_parameter("mtr.history_steps", 11);
  this->declare_parameter("mtr.history_rate_hz", 10.0);
  RCLCPP_INFO(this->get_logger(), "PredictionMlNode created (unconfigured)");
}

MtrConfig PredictionMlNode::loadMtrConfig()
{
  MtrConfig cfg;
  cfg.mode = parseMtrMode(this->get_parameter("mtr.mode").as_string());
  cfg.engine_path = this->get_parameter("mtr.engine_path").as_string();
  cfg.metadata_path = this->get_parameter("mtr.metadata_path").as_string();
  cfg.cache_ttl_s = this->get_parameter("mtr.cache_ttl_s").as_double();
  cfg.selected_target_agent_limit = static_cast<int>(this->get_parameter("mtr.selected_target_agent_limit").as_int());
  cfg.target_forward_half_angle_deg = this->get_parameter("mtr.target_forward_half_angle_deg").as_double();
  cfg.ego_length = this->get_parameter("mtr.ego_length").as_double();
  cfg.ego_width = this->get_parameter("mtr.ego_width").as_double();
  cfg.ego_height = this->get_parameter("mtr.ego_height").as_double();
  cfg.history_steps = static_cast<int>(this->get_parameter("mtr.history_steps").as_int());
  cfg.history_rate_hz = this->get_parameter("mtr.history_rate_hz").as_double();
  return cfg;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_configure(const rclcpp_lifecycle::State &)
{
  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
  prediction_time_step_ = this->get_parameter("prediction_time_step").as_double();
  fallback_vehicle_size_threshold_m_ = this->get_parameter("fallback.vehicle_size_threshold_m").as_double();
  fallback_vehicle_speed_mps_ = this->get_parameter("fallback.vehicle_speed_mps").as_double();
  fallback_vru_speed_mps_ = this->get_parameter("fallback.vru_speed_mps").as_double();
  ego_velocity_child_frame_ = this->get_parameter("mtr.ego_velocity_child_frame").as_string();

  const MtrConfig cfg = loadMtrConfig();
  const bool valid_config =
    std::isfinite(prediction_horizon_) && prediction_horizon_ > 0.0 && std::isfinite(prediction_time_step_) &&
    prediction_time_step_ > 0.0 && std::isfinite(fallback_vehicle_size_threshold_m_) &&
    fallback_vehicle_size_threshold_m_ >= 0.0 && std::isfinite(fallback_vehicle_speed_mps_) &&
    fallback_vehicle_speed_mps_ >= 0.0 && std::isfinite(fallback_vru_speed_mps_) && fallback_vru_speed_mps_ >= 0.0 &&
    std::isfinite(cfg.cache_ttl_s) && cfg.cache_ttl_s >= 0.0 && cfg.selected_target_agent_limit >= 0 &&
    std::isfinite(cfg.target_forward_half_angle_deg) && std::isfinite(cfg.ego_length) && cfg.ego_length > 0.0 &&
    std::isfinite(cfg.ego_width) && cfg.ego_width > 0.0 && std::isfinite(cfg.ego_height) && cfg.ego_height > 0.0 &&
    !ego_velocity_child_frame_.empty() && cfg.history_steps > 0 && std::isfinite(cfg.history_rate_hz) &&
    cfg.history_rate_hz > 0.0;
  if (!valid_config) {
    RCLCPP_ERROR(this->get_logger(), "Invalid prediction_ml timing, fallback, or MTR parameters");
    return CallbackReturn::FAILURE;
  }

  scene_builder_ = std::make_unique<SceneBuilder>(cfg);
  runtime_ = std::make_unique<MtrRuntime>(cfg);

  world_objects_pub_ = this->create_publisher<world_model_msgs::msg::WorldObjectArray>("world_object_seeds", 10);
  RCLCPP_INFO(this->get_logger(), "Configured (horizon=%.1fs, step=%.2fs)", prediction_horizon_, prediction_time_step_);
  if (!runtime_->ready()) {
    RCLCPP_WARN(this->get_logger(), "MTR unavailable; publishing fallback only: %s", runtime_->lastError().c_str());
  }
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_activate(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "tracks_3d", 10, std::bind(&PredictionMlNode::trackedObjectsCallback, this, std::placeholders::_1));
  ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "ego_pose", 10, std::bind(&PredictionMlNode::egoPoseCallback, this, std::placeholders::_1));
  ego_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "ego_odometry", 10, std::bind(&PredictionMlNode::egoOdometryCallback, this, std::placeholders::_1));
  lanelet_ahead_sub_ = this->create_subscription<lanelet_msgs::msg::LaneletAhead>(
    "lanelet_ahead", 10, std::bind(&PredictionMlNode::laneletAheadCallback, this, std::placeholders::_1));
  world_objects_pub_->on_activate();
  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  ego_odometry_sub_.reset();
  lanelet_ahead_sub_.reset();
  ego_pose_.reset();
  ego_odometry_.reset();
  lanelet_ahead_.reset();
  world_objects_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  ego_odometry_sub_.reset();
  lanelet_ahead_sub_.reset();
  world_objects_pub_.reset();
  scene_builder_.reset();
  runtime_.reset();
  ego_pose_.reset();
  ego_odometry_.reset();
  lanelet_ahead_.reset();
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  ego_odometry_sub_.reset();
  lanelet_ahead_sub_.reset();
  if (world_objects_pub_) {
    world_objects_pub_->on_deactivate();
    world_objects_pub_.reset();
  }
  scene_builder_.reset();
  runtime_.reset();
  ego_pose_.reset();
  ego_odometry_.reset();
  lanelet_ahead_.reset();
  return CallbackReturn::SUCCESS;
}

void PredictionMlNode::egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ego_pose_ = msg;
}

void PredictionMlNode::egoOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & velocity = msg->twist.twist.linear;
  if (
    msg->child_frame_id != ego_velocity_child_frame_ || !std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
    !std::isfinite(velocity.z))
  {
    ego_odometry_.reset();
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Ignoring ego odometry: expected finite twist in child_frame_id '%s', got '%s'",
      ego_velocity_child_frame_.c_str(),
      msg->child_frame_id.c_str());
    return;
  }
  ego_odometry_ = msg;
}

void PredictionMlNode::laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg)
{
  lanelet_ahead_ = msg;
}

std::vector<world_model_msgs::msg::WorldObject> PredictionMlNode::buildFallback(
  const vision_msgs::msg::Detection3DArray & msg, const rclcpp::Time & source_time) const
{
  std::vector<world_model_msgs::msg::WorldObject> objects;
  const std::string & frame_id = msg.header.frame_id;
  for (const auto & detection : msg.detections) {
    world_model_msgs::msg::WorldObject obj;
    obj.header = msg.header;
    obj.detection = detection;

    const double x = detection.bbox.center.position.x;
    const double y = detection.bbox.center.position.y;
    const double z = detection.bbox.center.position.z;
    const auto & q = detection.bbox.center.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double speed = selectFallbackSpeed(
      detection.bbox.size.x, fallback_vehicle_size_threshold_m_, fallback_vehicle_speed_mps_, fallback_vru_speed_mps_);

    world_model_msgs::msg::Prediction pred;
    pred.header = msg.header;
    pred.header.stamp = source_time;
    pred.conf = 1.0;
    const int num_steps = static_cast<int>(std::round(prediction_horizon_ / prediction_time_step_));
    for (int i = 1; i <= num_steps; ++i) {
      const double t = i * prediction_time_step_;
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = frame_id;
      ps.header.stamp = source_time + rclcpp::Duration::from_seconds(t);
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
  const rclcpp::Time now = this->get_clock()->now();
  const double now_s = now.seconds();
  const bool has_source_stamp = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0U;
  const rclcpp::Time source_time = has_source_stamp ? rclcpp::Time(msg->header.stamp) : now;

  auto fallback = buildFallback(*msg, source_time);

  scene_builder_->addFrame(*msg);
  MtrFrameContext frame;
  frame.detections = *msg;
  if (ego_pose_) {
    frame.ego_pose = *ego_pose_;
    frame.has_ego = true;
  }
  if (ego_odometry_) {
    frame.ego_velocity = ego_odometry_->twist.twist;
    frame.has_ego_velocity = true;
  }
  if (lanelet_ahead_) {
    frame.lanelet_ahead = *lanelet_ahead_;
    frame.has_map = true;
  }
  frame.timestamp = now_s;
  MtrInputTensors tensors = scene_builder_->build(frame);
  runtime_->submitFrame(
    std::move(tensors), msg->header.frame_id, source_time.seconds(), prediction_horizon_, prediction_time_step_);

  world_model_msgs::msg::WorldObjectArray output;
  output.header = msg->header;
  output.objects = runtime_->selectOutput(fallback, now_s);
  if (world_objects_pub_ && world_objects_pub_->is_activated()) {
    world_objects_pub_->publish(output);
  }
}

}  // namespace prediction_ml

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<prediction_ml::PredictionMlNode>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
