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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "prediction_ml/fallback_prediction.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace prediction_ml
{

namespace
{

constexpr double kOdometryBufferS = 1.0;  // odometry kept for track stamps that lag behind it

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

}  // namespace

vision_msgs::msg::Detection3DArray transformTracks(
  const vision_msgs::msg::Detection3DArray & tracks, const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Quaternion rotation;
  tf2::fromMsg(transform.transform.rotation, rotation);

  vision_msgs::msg::Detection3DArray out = tracks;
  out.header.frame_id = transform.header.frame_id;
  for (auto & detection : out.detections) {
    detection.header.frame_id = out.header.frame_id;
    const auto source = detection.bbox.center;
    tf2::doTransform(source, detection.bbox.center, transform);

    // ponytail: velocity is rotated only, never ego-motion compensated, so with tracking in a moving
    // frame (lidar_cc) parked cars keep their sensor-relative velocity (about -ego speed; tracking's
    // velocity is m/s since it passes ByteTrack the real dt). Only MTR inputs with
    // mtr.velocity_from_history false read it. Run tracking with output_frame map when absolute
    // velocities matter. Covariance untouched.
    for (auto & result : detection.results) {
      if (result.hypothesis.class_id != "linear_velocity") {
        continue;
      }
      auto & velocity = result.pose.pose.position;
      const tf2::Vector3 v = tf2::quatRotate(rotation, tf2::Vector3(velocity.x, velocity.y, velocity.z));
      velocity.x = v.x();
      velocity.y = v.y();
      velocity.z = v.z();
    }
  }
  return out;
}

world_model_msgs::msg::WorldObjectArray transformSeeds(
  const world_model_msgs::msg::WorldObjectArray & seeds, const geometry_msgs::msg::TransformStamped & transform)
{
  vision_msgs::msg::Detection3DArray detections;
  detections.header = seeds.header;
  for (const auto & object : seeds.objects) {
    detections.detections.push_back(object.detection);
  }
  detections = transformTracks(detections, transform);

  world_model_msgs::msg::WorldObjectArray out = seeds;
  const std::string & frame = transform.header.frame_id;
  out.header.frame_id = frame;
  for (std::size_t i = 0; i < out.objects.size(); ++i) {
    auto & object = out.objects[i];
    object.header.frame_id = frame;
    object.detection = detections.detections[i];
    for (auto & prediction : object.predictions) {
      prediction.header.frame_id = frame;
      for (auto & pose : prediction.poses) {
        pose.header.frame_id = frame;  // future stamps stay
        const auto source = pose.pose;
        tf2::doTransform(source, pose.pose, transform);
      }
    }
  }
  return out;
}

lanelet_msgs::msg::LaneletAhead transformLanes(
  const lanelet_msgs::msg::LaneletAhead & lanes, const geometry_msgs::msg::TransformStamped & transform)
{
  lanelet_msgs::msg::LaneletAhead out = lanes;
  out.header.frame_id = transform.header.frame_id;
  for (auto & lanelet : out.lanelets) {
    for (auto & point : lanelet.centerline) {
      const auto source = point;
      tf2::doTransform(source, point, transform);
    }
  }
  return out;
}

std::optional<lanelet_msgs::msg::LaneletAhead> lanesForFrame(
  const lanelet_msgs::msg::LaneletAhead::SharedPtr & lanes,
  const double track_time_s,
  const double max_age_s,
  const std::optional<geometry_msgs::msg::TransformStamped> & map_from_lanes)
{
  if (!lanes || !map_from_lanes || std::abs(stampSeconds(lanes->header.stamp) - track_time_s) > max_age_s) {
    return std::nullopt;
  }
  return transformLanes(*lanes, *map_from_lanes);
}

std::optional<nav_msgs::msg::Odometry> odometryAt(
  const std::deque<nav_msgs::msg::Odometry::SharedPtr> & buffer, const double stamp_s, const double max_gap_s)
{
  const auto after = std::lower_bound(
    buffer.begin(), buffer.end(), stamp_s, [](const nav_msgs::msg::Odometry::SharedPtr & sample, const double t) {
      return stampSeconds(sample->header.stamp) < t;
    });
  if (after == buffer.begin() || after == buffer.end()) {
    // Before the first or after the last sample: the nearest one, if close enough.
    if (buffer.empty()) {
      return std::nullopt;
    }
    const auto & nearest = after == buffer.begin() ? *buffer.front() : *buffer.back();
    if (std::abs(stampSeconds(nearest.header.stamp) - stamp_s) > max_gap_s) {
      return std::nullopt;
    }
    return nearest;
  }

  const auto & a = **std::prev(after);
  const auto & b = **after;
  const double ta = stampSeconds(a.header.stamp);
  const double tb = stampSeconds(b.header.stamp);
  if (std::min(stamp_s - ta, tb - stamp_s) > max_gap_s) {
    return std::nullopt;
  }
  const double r = (stamp_s - ta) / (tb - ta);  // tb > ta: lower_bound put stamp_s in (ta, tb]
  const auto lerp = [r](const double x, const double y) { return x + (y - x) * r; };

  nav_msgs::msg::Odometry out = b;
  out.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(stamp_s * 1.0e9)));
  auto & position = out.pose.pose.position;
  position.x = lerp(a.pose.pose.position.x, b.pose.pose.position.x);
  position.y = lerp(a.pose.pose.position.y, b.pose.pose.position.y);
  position.z = lerp(a.pose.pose.position.z, b.pose.pose.position.z);
  tf2::Quaternion qa;
  tf2::Quaternion qb;
  tf2::fromMsg(a.pose.pose.orientation, qa);
  tf2::fromMsg(b.pose.pose.orientation, qb);
  out.pose.pose.orientation = tf2::toMsg(tf2::slerp(qa, qb, r));
  auto & linear = out.twist.twist.linear;
  linear.x = lerp(a.twist.twist.linear.x, b.twist.twist.linear.x);
  linear.y = lerp(a.twist.twist.linear.y, b.twist.twist.linear.y);
  linear.z = lerp(a.twist.twist.linear.z, b.twist.twist.linear.z);
  auto & angular = out.twist.twist.angular;
  angular.x = lerp(a.twist.twist.angular.x, b.twist.twist.angular.x);
  angular.y = lerp(a.twist.twist.angular.y, b.twist.twist.angular.y);
  angular.z = lerp(a.twist.twist.angular.z, b.twist.twist.angular.z);
  return out;
}

std::optional<ExactStampJoiner::Stamp> ExactStampJoiner::stampKey(
  const std::int32_t seconds, const std::uint32_t nanoseconds)
{
  if (seconds < 0 || nanoseconds >= 1000000000U || (seconds == 0 && nanoseconds == 0U)) {
    return std::nullopt;
  }
  return static_cast<Stamp>(seconds) * 1000000000LL + static_cast<Stamp>(nanoseconds);
}

std::optional<ExactStampInputs> ExactStampJoiner::addTracks(vision_msgs::msg::Detection3DArray::SharedPtr message)
{
  return insertAndTake(tracks_, std::move(message));
}

std::optional<ExactStampInputs> ExactStampJoiner::addOdometry(nav_msgs::msg::Odometry::SharedPtr message)
{
  return insertAndTake(odometry_, std::move(message));
}

std::optional<ExactStampInputs> ExactStampJoiner::addLanelet(lanelet_msgs::msg::LaneletAhead::SharedPtr message)
{
  return insertAndTake(lanelets_, std::move(message));
}

std::optional<ExactStampInputs> ExactStampJoiner::take(const Stamp stamp)
{
  const auto tracks = tracks_.find(stamp);
  const auto odometry = odometry_.find(stamp);
  const auto lanelet = lanelets_.find(stamp);
  if (tracks == tracks_.end() || odometry == odometry_.end() || lanelet == lanelets_.end()) {
    return std::nullopt;
  }

  ExactStampInputs inputs{tracks->second, odometry->second, lanelet->second};
  tracks_.erase(tracks);
  odometry_.erase(odometry);
  lanelets_.erase(lanelet);
  rememberProcessed(stamp);
  return inputs;
}

void ExactStampJoiner::rememberProcessed(const Stamp stamp)
{
  processed_.insert(stamp);
  processed_order_.push_back(stamp);
  while (processed_order_.size() > kCapacity) {
    processed_.erase(processed_order_.front());
    processed_order_.pop_front();
  }
}

void ExactStampJoiner::clear()
{
  tracks_.clear();
  odometry_.clear();
  lanelets_.clear();
  processed_order_.clear();
  processed_.clear();
}

PredictionMlNode::PredictionMlNode(const rclcpp::NodeOptions & options)
: LifecycleNode("prediction_ml_node", options)
{
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odometry_topic", "/interfacing/can_state_estimator/odom");
  this->declare_parameter("odometry_max_gap_s", 0.2);
  this->declare_parameter("prediction_horizon", 3.0);
  this->declare_parameter("prediction_time_step", 0.2);
  this->declare_parameter("mtr.mode", "disabled");
  this->declare_parameter("mtr.onnx_path", "");
  this->declare_parameter("mtr.engine_path", "");
  this->declare_parameter("mtr.map_onnx_path", "");
  this->declare_parameter("mtr.map_engine_path", "");
  this->declare_parameter("mtr.metadata_path", "");
  this->declare_parameter("mtr.cache_ttl_s", 0.5);
  this->declare_parameter("mtr.selected_target_agent_limit", 8);
  this->declare_parameter("mtr.target_forward_half_angle_deg", 90.0);
  this->declare_parameter<std::vector<std::string>>("mtr.pinned_target_ids", std::vector<std::string>{});
  this->declare_parameter("mtr.model_time_step_s", 0.1);
  this->declare_parameter("mtr.ego_length", 4.0936);
  this->declare_parameter("mtr.ego_width", 1.3579);
  this->declare_parameter("mtr.ego_height", 1.4090);
  this->declare_parameter("mtr.ego_velocity_child_frame", "base_footprint");
  this->declare_parameter("mtr.history_steps", 11);
  this->declare_parameter("mtr.history_rate_hz", 10.0);
  this->declare_parameter("mtr.velocity_from_history", false);
  this->declare_parameter("mtr.planar_inputs", false);
  this->declare_parameter("mtr.moving_targets_only", false);
  RCLCPP_INFO(this->get_logger(), "PredictionMlNode created (unconfigured)");
}

MtrConfig PredictionMlNode::loadMtrConfig()
{
  MtrConfig cfg;
  try {
    cfg.mode = parseMtrMode(this->get_parameter("mtr.mode").as_string());
  } catch (const std::invalid_argument & error) {
    // A typo must not take down the only seeds publisher: be loud, then run fallback-only.
    RCLCPP_ERROR(this->get_logger(), "%s; publishing fallback only", error.what());
    cfg.mode = MtrMode::Disabled;
  }
  cfg.onnx_path = this->get_parameter("mtr.onnx_path").as_string();
  cfg.engine_path = this->get_parameter("mtr.engine_path").as_string();
  cfg.map_onnx_path = this->get_parameter("mtr.map_onnx_path").as_string();
  cfg.map_engine_path = this->get_parameter("mtr.map_engine_path").as_string();
  cfg.metadata_path = this->get_parameter("mtr.metadata_path").as_string();
  cfg.cache_ttl_s = this->get_parameter("mtr.cache_ttl_s").as_double();
  cfg.selected_target_agent_limit = static_cast<int>(this->get_parameter("mtr.selected_target_agent_limit").as_int());
  cfg.target_forward_half_angle_deg = this->get_parameter("mtr.target_forward_half_angle_deg").as_double();
  cfg.pinned_target_ids = this->get_parameter("mtr.pinned_target_ids").as_string_array();
  cfg.model_time_step_s = this->get_parameter("mtr.model_time_step_s").as_double();
  cfg.ego_length = this->get_parameter("mtr.ego_length").as_double();
  cfg.ego_width = this->get_parameter("mtr.ego_width").as_double();
  cfg.ego_height = this->get_parameter("mtr.ego_height").as_double();
  cfg.history_steps = static_cast<int>(this->get_parameter("mtr.history_steps").as_int());
  cfg.history_rate_hz = this->get_parameter("mtr.history_rate_hz").as_double();
  cfg.velocity_from_history = this->get_parameter("mtr.velocity_from_history").as_bool();
  cfg.planar_inputs = this->get_parameter("mtr.planar_inputs").as_bool();
  cfg.moving_targets_only = this->get_parameter("mtr.moving_targets_only").as_bool();
  return cfg;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_configure(const rclcpp_lifecycle::State &)
{
  map_frame_ = this->get_parameter("map_frame").as_string();
  odometry_topic_ = this->get_parameter("odometry_topic").as_string();
  odometry_max_gap_s_ = this->get_parameter("odometry_max_gap_s").as_double();
  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
  prediction_time_step_ = this->get_parameter("prediction_time_step").as_double();
  ego_velocity_child_frame_ = this->get_parameter("mtr.ego_velocity_child_frame").as_string();

  const MtrConfig cfg = loadMtrConfig();
  const bool valid_model_time_step = std::isfinite(cfg.model_time_step_s) && cfg.model_time_step_s > 0.0;
  const double cadence_ratio = valid_model_time_step ? prediction_time_step_ / cfg.model_time_step_s : 0.0;
  const bool valid_prediction_cadence = std::isfinite(prediction_time_step_) && prediction_time_step_ > 0.0 &&
                                        std::isfinite(cadence_ratio) && cadence_ratio >= 1.0 &&
                                        std::abs(cadence_ratio - std::round(cadence_ratio)) <= 1.0e-9;
  const bool valid_config = std::isfinite(prediction_horizon_) && prediction_horizon_ > 0.0 &&
                            valid_prediction_cadence && std::isfinite(cfg.cache_ttl_s) && cfg.cache_ttl_s >= 0.0 &&
                            cfg.selected_target_agent_limit >= 0 && std::isfinite(cfg.target_forward_half_angle_deg) &&
                            valid_model_time_step && std::isfinite(cfg.ego_length) && cfg.ego_length > 0.0 &&
                            std::isfinite(cfg.ego_width) && cfg.ego_width > 0.0 && std::isfinite(cfg.ego_height) &&
                            cfg.ego_height > 0.0 && !ego_velocity_child_frame_.empty() && !map_frame_.empty() &&
                            cfg.history_steps > 0 && std::isfinite(cfg.history_rate_hz) && cfg.history_rate_hz > 0.0;
  const bool valid_odometry =
    !odometry_topic_.empty() && std::isfinite(odometry_max_gap_s_) && odometry_max_gap_s_ >= 0.0;
  if (!valid_config || !valid_odometry) {
    RCLCPP_ERROR(this->get_logger(), "Invalid prediction_ml timing, fallback, or MTR parameters");
    return CallbackReturn::FAILURE;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  lane_max_age_s_ = cfg.cache_ttl_s;
  scene_builder_ = std::make_unique<SceneBuilder>(cfg);
  const std::array<std::pair<std::string, std::string>, 2> engines{
    {{cfg.onnx_path, cfg.engine_path}, {cfg.map_onnx_path, cfg.map_engine_path}}};
  for (const auto & [onnx, engine] : engines) {
    if (cfg.mode == MtrMode::TensorRt && !onnx.empty() && !engine.empty() && !std::filesystem::exists(engine)) {
      RCLCPP_INFO(
        this->get_logger(),
        "Building TensorRT engine %s from %s (takes minutes; reused after)",
        engine.c_str(),
        onnx.c_str());
    }
  }
  runtime_ = std::make_unique<MtrRuntime>(cfg);
  mtr_diagnostics_enabled_ = !cfg.pinned_target_ids.empty();

  world_objects_pub_ = this->create_publisher<world_model_msgs::msg::WorldObjectArray>("world_object_seeds", 10);
  mtr_predictions_pub_ = this->create_publisher<world_model_msgs::msg::WorldObjectArray>("mtr_predictions", 10);
  RCLCPP_INFO(this->get_logger(), "Configured (horizon=%.1fs, step=%.2fs)", prediction_horizon_, prediction_time_step_);
  if (!runtime_->ready() && cfg.mode == MtrMode::TensorRt) {
    // TensorRT was asked for (bad engine, or a build without the backend): that is an error, not a mode.
    RCLCPP_ERROR(this->get_logger(), "MTR unavailable; publishing fallback only: %s", runtime_->lastError().c_str());
  } else if (!runtime_->ready()) {
    RCLCPP_WARN(this->get_logger(), "MTR unavailable; publishing fallback only: %s", runtime_->lastError().c_str());
  } else if (!cfg.map_engine_path.empty() && !runtime_->mapReady()) {
    RCLCPP_WARN(
      this->get_logger(), "MTR map engine unavailable; every frame runs no-map: %s", runtime_->mapError().c_str());
  }
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_activate(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "tracks_3d", 10, std::bind(&PredictionMlNode::trackedObjectsCallback, this, std::placeholders::_1));
  ego_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, 50, std::bind(&PredictionMlNode::egoOdometryCallback, this, std::placeholders::_1));
  lanelet_ahead_sub_ = this->create_subscription<lanelet_msgs::msg::LaneletAhead>(
    "lanelet_ahead", 10, std::bind(&PredictionMlNode::laneletAheadCallback, this, std::placeholders::_1));
  world_objects_pub_->on_activate();
  mtr_predictions_pub_->on_activate();
  if (mtr_diagnostics_enabled_) {
    diagnostic_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PredictionMlNode::publishMtrDiagnostics, this));
  }
  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_odometry_sub_.reset();
  lanelet_ahead_sub_.reset();
  odometry_buffer_.clear();
  lanelet_ahead_.reset();
  exact_stamp_joiner_.clear();
  diagnostic_timer_.reset();
  world_objects_pub_->on_deactivate();
  mtr_predictions_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_odometry_sub_.reset();
  lanelet_ahead_sub_.reset();
  diagnostic_timer_.reset();
  runtime_.reset();
  world_objects_pub_.reset();
  mtr_predictions_pub_.reset();
  scene_builder_.reset();
  exact_stamp_joiner_.clear();
  mtr_diagnostics_enabled_ = false;
  odometry_buffer_.clear();
  lanelet_ahead_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_odometry_sub_.reset();
  lanelet_ahead_sub_.reset();
  diagnostic_timer_.reset();
  runtime_.reset();
  if (world_objects_pub_) {
    world_objects_pub_->on_deactivate();
    world_objects_pub_.reset();
  }
  if (mtr_predictions_pub_) {
    mtr_predictions_pub_->on_deactivate();
    mtr_predictions_pub_.reset();
  }
  scene_builder_.reset();
  exact_stamp_joiner_.clear();
  mtr_diagnostics_enabled_ = false;
  odometry_buffer_.clear();
  lanelet_ahead_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  return CallbackReturn::SUCCESS;
}

void PredictionMlNode::egoOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto finite = [](const auto & v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); };
  const auto & orientation = msg->pose.pose.orientation;
  if (
    msg->header.frame_id != map_frame_ || msg->child_frame_id != ego_velocity_child_frame_ ||
    !finite(msg->pose.pose.position) || !finite(orientation) || !std::isfinite(orientation.w) ||
    !finite(msg->twist.twist.linear) || !finite(msg->twist.twist.angular) || msg->header.stamp.sec < 0)
  {
    odometry_buffer_.clear();
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Ignoring ego odometry: expected a finite '%s' -> '%s' pose and twist, got '%s' -> '%s'",
      map_frame_.c_str(),
      ego_velocity_child_frame_.c_str(),
      msg->header.frame_id.c_str(),
      msg->child_frame_id.c_str());
    return;
  }
  // A stamp going backwards (bag loop, clock reset) starts a new buffer.
  if (
    !odometry_buffer_.empty() && stampSeconds(msg->header.stamp) < stampSeconds(odometry_buffer_.back()->header.stamp))
  {
    odometry_buffer_.clear();
  }
  odometry_buffer_.push_back(msg);
  while (stampSeconds(msg->header.stamp) - stampSeconds(odometry_buffer_.front()->header.stamp) > kOdometryBufferS) {
    odometry_buffer_.pop_front();
  }
  if (mtr_diagnostics_enabled_) {
    if (auto inputs = exact_stamp_joiner_.addOdometry(msg)) {
      submitSynchronizedMtr(std::move(*inputs));
    }
  }
}

void PredictionMlNode::laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg)
{
  if (mtr_diagnostics_enabled_) {
    if (auto inputs = exact_stamp_joiner_.addLanelet(msg)) {
      submitSynchronizedMtr(std::move(*inputs));
    }
    return;
  }
  lanelet_ahead_ = msg;
}

void PredictionMlNode::publishMtrDiagnostics()
{
  if (!runtime_ || !mtr_predictions_pub_ || !mtr_predictions_pub_->is_activated()) {
    return;
  }
  while (auto output = runtime_->takeDiagnosticOutput()) {
    mtr_predictions_pub_->publish(std::move(*output));
  }
}

void PredictionMlNode::submitSynchronizedMtr(ExactStampInputs inputs)
{
  if (!inputs.tracks || !inputs.odometry || !inputs.lanelet) {
    return;
  }

  MtrFrameContext frame;
  frame.detections = *inputs.tracks;
  frame.ego_pose.header = inputs.odometry->header;
  frame.ego_pose.pose = inputs.odometry->pose.pose;
  frame.ego_velocity = inputs.odometry->twist.twist;
  frame.lanelet_ahead = *inputs.lanelet;
  frame.timestamp = rclcpp::Time(inputs.tracks->header.stamp).seconds();
  frame.has_ego = true;
  frame.has_ego_velocity = true;
  frame.has_map = true;

  MtrInputTensors tensors = scene_builder_->build(frame);
  runtime_->submitFrame(
    std::move(tensors),
    inputs.tracks->header.frame_id,
    frame.timestamp,
    prediction_horizon_,
    prediction_time_step_,
    frame.has_map);
}

std::vector<world_model_msgs::msg::WorldObject> PredictionMlNode::buildFallback(
  const vision_msgs::msg::Detection3DArray & msg, const rclcpp::Time & source_time) const
{
  std::vector<world_model_msgs::msg::WorldObject> objects;
  const std::string & frame_id = msg.header.frame_id;
  // Measured velocities live in map_frame_; tracks left in their sensor frame (no TF) use the tracker's.
  const bool in_map_frame = frame_id.empty() || frame_id == map_frame_;
  for (const auto & detection : msg.detections) {
    world_model_msgs::msg::WorldObject obj;
    obj.header = msg.header;
    obj.detection = detection;

    const double x = detection.bbox.center.position.x;
    const double y = detection.bbox.center.position.y;
    const double z = detection.bbox.center.position.z;
    // Velocity measured from the track's own history (parked cars, barriers and tracks younger than
    // 0.3 s measure 0 and get a zero-length path). Tracks left in their sensor frame use the tracker's
    // linear_velocity, or a zero-length path without one.
    const auto velocity =
      (scene_builder_ && in_map_frame) ? scene_builder_->measuredVelocity(detection.id) : trackerVelocity(detection);
    const double vx = velocity[0];
    const double vy = velocity[1];

    world_model_msgs::msg::Prediction pred;
    pred.header = msg.header;
    pred.header.stamp = source_time;
    pred.conf = 1.0;
    // Same step count as MTR (output_converter.cpp): never past the horizon.
    const int num_steps = static_cast<int>(
      std::floor(prediction_horizon_ / prediction_time_step_ + std::numeric_limits<double>::epsilon() * 8.0));
    for (int i = 1; i <= num_steps; ++i) {
      const double t = i * prediction_time_step_;
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = frame_id;
      ps.header.stamp = source_time + rclcpp::Duration::from_seconds(t);
      ps.pose.position.x = x + vx * t;
      ps.pose.position.y = y + vy * t;
      ps.pose.position.z = z;
      ps.pose.orientation = detection.bbox.center.orientation;
      pred.poses.push_back(ps);
    }
    obj.predictions.push_back(pred);
    objects.push_back(obj);
  }
  return objects;
}

std::optional<geometry_msgs::msg::TransformStamped> PredictionMlNode::mapFrom(
  const std::string & frame, const builtin_interfaces::msg::Time & stamp, const nav_msgs::msg::Odometry & ego)
{
  try {
    // At the track stamp (an unstamped message gets the latest); no wait, a late transform is a miss.
    const auto ego_from_frame_msg = tf_buffer_->lookupTransform(ego.child_frame_id, frame, tf2_ros::fromMsg(stamp));
    tf2::Transform map_from_ego;
    tf2::fromMsg(ego.pose.pose, map_from_ego);
    tf2::Transform ego_from_frame;
    tf2::fromMsg(ego_from_frame_msg.transform, ego_from_frame);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = map_frame_;
    transform.header.stamp = stamp;
    transform.child_frame_id = frame;
    transform.transform = tf2::toMsg(map_from_ego * ego_from_frame);
    return transform;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "No %s <- %s transform at the track stamp (tracks: fallback only; lanes: no-map engine): %s",
      ego.child_frame_id.c_str(),
      frame.c_str(),
      ex.what());
    return std::nullopt;
  }
}

void PredictionMlNode::trackedObjectsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr raw_msg)
{
  // rclcpp::Time throws on a negative stamp, which would terminate the node from this callback.
  if (raw_msg->header.stamp.sec < 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Dropping tracks with negative stamp %d",
      raw_msg->header.stamp.sec);
    return;
  }
  const rclcpp::Time now = this->get_clock()->now();
  const bool has_source_stamp = raw_msg->header.stamp.sec != 0 || raw_msg->header.stamp.nanosec != 0U;
  const rclcpp::Time source_time = has_source_stamp ? rclcpp::Time(raw_msg->header.stamp) : now;
  // MTR results age on the track clock (like world_model's permanence), so sim and bag stamps work
  // whether or not use_sim_time is set.
  const double track_now_s = source_time.seconds();

  // Ego pose and velocity at the track stamp, from odometry in map_frame_.
  const auto ego = odometryAt(odometry_buffer_, track_now_s, odometry_max_gap_s_);
  if (!ego) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "No %s sample within odometry_max_gap_s (%.2f s) of the tracks; publishing fallback only",
      odometry_topic_.c_str(),
      odometry_max_gap_s_);
  }

  // Ego pose, the lanelet map and MTR inputs are all in map_frame_; tracking may publish in a
  // sensor frame (perception_bringup uses lidar_cc). Seeds go out in the tracks' own frame, which
  // world_model resolves through the TF tree; map_frame_ (a topic-only odometry) may not be in it.
  auto msg = raw_msg;
  std::optional<geometry_msgs::msg::TransformStamped> tracks_from_map;
  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    const auto map_from_tracks = ego ? mapFrom(msg->header.frame_id, msg->header.stamp, *ego)
                                     : std::optional<geometry_msgs::msg::TransformStamped>{};
    if (!map_from_tracks) {
      // No odometry or mount transform: publish the fallback, not nothing.
      world_model_msgs::msg::WorldObjectArray output;
      output.header = raw_msg->header;
      output.objects = buildFallback(*raw_msg, source_time);
      if (world_objects_pub_ && world_objects_pub_->is_activated()) {
        world_objects_pub_->publish(output);
      }
      return;
    }
    msg = std::make_shared<vision_msgs::msg::Detection3DArray>(transformTracks(*msg, *map_from_tracks));
    tf2::Transform map_from_tracks_tf;
    tf2::fromMsg(map_from_tracks->transform, map_from_tracks_tf);
    tracks_from_map = *map_from_tracks;
    tracks_from_map->header.frame_id = raw_msg->header.frame_id;
    tracks_from_map->child_frame_id = map_frame_;
    tracks_from_map->transform = tf2::toMsg(map_from_tracks_tf.inverse());
  }
  const auto publish = [this, &tracks_from_map](const world_model_msgs::msg::WorldObjectArray & output) {
    if (world_objects_pub_ && world_objects_pub_->is_activated()) {
      world_objects_pub_->publish(tracks_from_map ? transformSeeds(output, *tracks_from_map) : output);
    }
  };

  scene_builder_->addFrame(*msg);  // first, so the fallback sees this frame's measured velocity
  auto fallback = buildFallback(*msg, source_time);
  if (mtr_diagnostics_enabled_) {
    // MTR is submitted only from the exact-stamp join; this frame's tensors may not exist yet.
    auto synchronized = exact_stamp_joiner_.addTracks(msg);

    world_model_msgs::msg::WorldObjectArray output;
    output.header = msg->header;
    output.objects = runtime_->selectOutput(fallback, track_now_s);
    publish(output);

    if (synchronized) {
      submitSynchronizedMtr(std::move(*synchronized));
    }
    return;
  }

  MtrFrameContext frame;
  frame.detections = *msg;
  // Missing or stale odometry gets no MTR this frame (fallback only).
  if (ego) {
    frame.ego_pose.header = ego->header;
    frame.ego_pose.pose = ego->pose.pose;
    frame.ego_velocity = ego->twist.twist;
    frame.has_ego = true;
    frame.has_ego_velocity = true;
  }
  // Fresh lanes, moved into map_frame_ like the tracks, run the map engine; every other frame runs the
  // no-map engine with no lanes, as it was trained.
  if (ego && lanelet_ahead_ && runtime_->mapReady()) {
    std::optional<geometry_msgs::msg::TransformStamped> map_from_lanes;
    if (lanelet_ahead_->header.frame_id == map_frame_) {
      map_from_lanes.emplace();
      map_from_lanes->header.frame_id = map_frame_;
      map_from_lanes->transform.rotation.w = 1.0;
    } else {
      map_from_lanes = mapFrom(lanelet_ahead_->header.frame_id, raw_msg->header.stamp, *ego);
    }
    if (auto lanes = lanesForFrame(lanelet_ahead_, track_now_s, lane_max_age_s_, map_from_lanes)) {
      frame.lanelet_ahead = std::move(*lanes);
      frame.has_map = true;
    }
  }
  if (runtime_->ready()) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000, "MTR engine: %s", frame.has_map ? "map (fresh lanes)" : "no-map");
  }
  frame.timestamp = track_now_s;
  MtrInputTensors tensors = scene_builder_->build(frame);
  runtime_->submitFrame(
    std::move(tensors),
    msg->header.frame_id,
    source_time.seconds(),
    prediction_horizon_,
    prediction_time_step_,
    frame.has_map);

  world_model_msgs::msg::WorldObjectArray output;
  output.header = msg->header;
  output.objects = runtime_->selectOutput(fallback, track_now_s);
  publish(output);
}

}  // namespace prediction_ml

#ifndef PREDICTION_ML_DISABLE_MAIN
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
#endif
