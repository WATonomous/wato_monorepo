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

#include "prediction/trajectory_predictor.hpp"

#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

namespace prediction
{

namespace
{
// Extract yaw from quaternion
double extractYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Build KinematicState from detection
KinematicState stateFromDetection(
  const geometry_msgs::msg::Point & position, const geometry_msgs::msg::Quaternion & orientation, double velocity)
{
  KinematicState state;
  state.x = position.x;
  state.y = position.y;
  state.theta = extractYaw(orientation);
  state.v = velocity;
  state.a = 0.0;
  state.delta = 0.0;
  return state;
}

// Build TrajectoryHypothesis from motion model output (move semantics for
// efficiency)
TrajectoryHypothesis buildHypothesis(
  std::vector<geometry_msgs::msg::Pose> && poses, double time_step, Intent intent = Intent::CONTINUE_STRAIGHT)
{
  TrajectoryHypothesis hypothesis;
  hypothesis.intent = intent;
  hypothesis.probability = 0.0;
  hypothesis.waypoints = std::move(poses);

  const size_t num_waypoints = hypothesis.waypoints.size();
  hypothesis.timestamps.reserve(num_waypoints);
  double t = time_step;
  for (size_t i = 0; i < num_waypoints; ++i) {
    hypothesis.timestamps.push_back(t);
    t += time_step;
  }

  return hypothesis;
}
}  // namespace

TrajectoryPredictor::TrajectoryPredictor(
  rclcpp_lifecycle::LifecycleNode * node,
  double prediction_horizon,
  double time_step,
  const CyclistParams & cyclist_params,
  std::shared_ptr<world_model::LaneletHandler> lanelet_handler)
: node_(node)
, prediction_horizon_(prediction_horizon)
, time_step_(time_step)
, cyclist_params_(cyclist_params)
, lanelet_handler_(lanelet_handler)
{
  bicycle_model_ = std::make_unique<BicycleModel>();
  constant_velocity_model_ = std::make_unique<ConstantVelocityModel>();

  RCLCPP_INFO(node_->get_logger(), "TrajectoryPredictor initialized");
}

void TrajectoryPredictor::setLaneletHandler(std::shared_ptr<world_model::LaneletHandler> lanelet_handler)
{
  std::unique_lock lock(lanelet_handler_mutex_);
  lanelet_handler_ = std::move(lanelet_handler);
  RCLCPP_INFO(node_->get_logger(), "LaneletHandler set for TrajectoryPredictor");
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateHypotheses(
  const vision_msgs::msg::Detection3D & detection, double timestamp)
{
  std::optional<double> velocity =
    updateHistoryAndComputeVelocity(detection.id, detection.bbox.center.position, timestamp);

  ObjectType obj_type = classifyObjectType(detection);

  switch (obj_type) {
    case ObjectType::VEHICLE:
      return generateVehicleHypotheses(detection, velocity);
    case ObjectType::PEDESTRIAN:
      return generatePedestrianHypotheses(detection, velocity);
    case ObjectType::CYCLIST:
      return generateCyclistHypotheses(detection, velocity);
    default:
      RCLCPP_WARN(node_->get_logger(), "Unknown object type, skipping prediction");
      return {};
  }
}

ObjectType TrajectoryPredictor::classifyObjectType(const vision_msgs::msg::Detection3D & detection) const
{
  double length = detection.bbox.size.x;
  double height = detection.bbox.size.z;

  if (length > 3.5) {
    return ObjectType::VEHICLE;
  } else if (length < 1.0 && height > 1.0) {
    return ObjectType::PEDESTRIAN;
  } else if (length >= 1.0 && length <= 3.5) {
    return ObjectType::CYCLIST;
  }

  return ObjectType::VEHICLE;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateVehicleHypotheses(
  const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity)
{
  // Use computed velocity if available, otherwise default to typical vehicle
  // speed
  // TODO(prediction): Move these defaults to params.yaml
  constexpr double kDefaultVehicleSpeed = 5.0;  // m/s
  constexpr double kMaxVehicleSpeed = 30.0;  // m/s

  double v = velocity.value_or(kDefaultVehicleSpeed);
  v = std::clamp(v, 0.0, kMaxVehicleSpeed);

  auto state = stateFromDetection(detection.bbox.center.position, detection.bbox.center.orientation, v);

  auto poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Vehicle prediction: velocity=%.2f m/s", v);

  return {buildHypothesis(std::move(poses), time_step_)};
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generatePedestrianHypotheses(
  const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity)
{
  // Use computed velocity if available, otherwise default to walking speed
  // TODO(prediction): Move these defaults to params.yaml
  constexpr double kDefaultWalkingSpeed = 1.4;  // m/s (average human walking speed)
  constexpr double kMaxPedestrianSpeed = 3.0;  // m/s (fast jog)

  double v = velocity.value_or(kDefaultWalkingSpeed);
  v = std::clamp(v, 0.0, kMaxPedestrianSpeed);

  auto state = stateFromDetection(detection.bbox.center.position, detection.bbox.center.orientation, v);

  auto poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Pedestrian prediction: velocity=%.2f m/s", v);

  return {buildHypothesis(std::move(poses), time_step_)};
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateCyclistHypotheses(
  const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity)
{
  std::vector<TrajectoryHypothesis> hypotheses;
  const auto & pos = detection.bbox.center.position;

  // Compute velocity with param-based clamping
  const double default_speed = (cyclist_params_.min_speed + cyclist_params_.max_speed) * 0.5;
  double v = velocity.value_or(default_speed);
  v = std::clamp(v, cyclist_params_.min_speed, cyclist_params_.max_speed);

  auto state = stateFromDetection(pos, detection.bbox.center.orientation, v);

  // Thread-safe access to lanelet handler
  std::shared_lock lock(lanelet_handler_mutex_);

  // Check if we can use lanelet-based prediction
  if (!lanelet_handler_ || !lanelet_handler_->isMapLoaded() || !lanelet_handler_->getRoutingGraph()) {
    lock.unlock();  // Release lock before computation
    auto poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);
    auto hypothesis = buildHypothesis(std::move(poses), time_step_, Intent::CONTINUE_STRAIGHT);
    hypothesis.probability = 1.0;
    hypotheses.push_back(std::move(hypothesis));

    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "Cyclist prediction: constant-velocity (no map), velocity=%.2f m/s",
      v);
    return hypotheses;
  }

  // Use LaneletHandler to find nearest lanelet
  auto nearest_lanelet = lanelet_handler_->findNearestLanelet(pos);
  if (!nearest_lanelet.has_value()) {
    lock.unlock();
    auto poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);
    auto hypothesis = buildHypothesis(std::move(poses), time_step_, Intent::CONTINUE_STRAIGHT);
    hypothesis.probability = 1.0;
    hypotheses.push_back(std::move(hypothesis));

    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "Cyclist prediction: constant-velocity (no lanelet), velocity=%.2f m/s",
      v);
    return hypotheses;
  }

  // Check if cyclist is within proximity threshold of the nearest lanelet
  // centerline
  const auto & centerline = nearest_lanelet->centerline();
  double min_dist_sq = std::numeric_limits<double>::max();
  for (const auto & pt : centerline) {
    const double dx = pt.x() - pos.x;
    const double dy = pt.y() - pos.y;
    min_dist_sq = std::min(min_dist_sq, dx * dx + dy * dy);
  }

  const double threshold_sq = cyclist_params_.lanelet_proximity_threshold * cyclist_params_.lanelet_proximity_threshold;
  if (min_dist_sq > threshold_sq) {
    lock.unlock();
    auto poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);
    auto hypothesis = buildHypothesis(std::move(poses), time_step_, Intent::CONTINUE_STRAIGHT);
    hypothesis.probability = 1.0;
    hypotheses.push_back(std::move(hypothesis));

    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "Cyclist prediction: constant-velocity (off-road), velocity=%.2f m/s",
      v);
    return hypotheses;
  }

  // Near lanelet - generate multi-hypothesis trajectories
  auto all_paths = getAllLaneletPaths(*nearest_lanelet, cyclist_params_.max_lanelet_search_depth);
  lock.unlock();  // Release lock before heavy computation

  if (all_paths.empty()) {
    auto poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);
    auto hypothesis = buildHypothesis(std::move(poses), time_step_, Intent::CONTINUE_STRAIGHT);
    hypothesis.probability = 1.0;
    hypotheses.push_back(std::move(hypothesis));

    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "Cyclist prediction: constant-velocity (no paths), velocity=%.2f m/s",
      v);
    return hypotheses;
  }

  // Reserve space for hypotheses (paths + 1 CV fallback)
  hypotheses.reserve(all_paths.size() + 1);

  // Distribute confidence among lanelet-based hypotheses (70% total)
  // Straight gets higher base confidence than turns
  constexpr double kLaneletTotalConf = 0.7;
  constexpr double kCvFallbackConf = 0.3;
  constexpr double kStraightBoost = 1.5;

  double total_weight = 0.0;
  for (const auto & [path, intent] : all_paths) {
    total_weight += (intent == Intent::CONTINUE_STRAIGHT) ? kStraightBoost : 1.0;
  }

  for (auto & [path_points, intent] : all_paths) {
    if (path_points.size() < 2) {
      continue;
    }

    auto poses = bicycle_model_->generateTrajectory(state, path_points, prediction_horizon_, time_step_);
    auto hypothesis = buildHypothesis(std::move(poses), time_step_, intent);

    const double weight = (intent == Intent::CONTINUE_STRAIGHT) ? kStraightBoost : 1.0;
    hypothesis.probability = kLaneletTotalConf * (weight / total_weight);

    hypotheses.push_back(std::move(hypothesis));
  }

  // Add constant velocity fallback with lower confidence
  auto cv_poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);
  auto cv_hypothesis = buildHypothesis(std::move(cv_poses), time_step_, Intent::UNKNOWN);
  cv_hypothesis.probability = kCvFallbackConf;
  hypotheses.push_back(std::move(cv_hypothesis));

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "Cyclist prediction: %zu lanelet-based + 1 CV "
    "fallback, velocity=%.2f m/s",
    all_paths.size(),
    v);

  return hypotheses;
}

std::optional<double> TrajectoryPredictor::updateHistoryAndComputeVelocity(
  const std::string & object_id, const geometry_msgs::msg::Point & position, double timestamp)
{
  std::lock_guard lock(history_mutex_);

  auto & history = detection_history_[object_id];

  // Prune old entries (O(1) with deque)
  while (!history.empty() && (timestamp - history.front().timestamp) > kHistoryMaxAge) {
    history.pop_front();
  }

  // Add current detection
  history.push_back({position, timestamp});

  // Limit history size (O(1) with deque)
  while (history.size() > kHistoryMaxSize) {
    history.pop_front();
  }

  // Need at least 2 points to compute velocity
  if (history.size() < 2) {
    return std::nullopt;
  }

  const auto & prev = history[history.size() - 2];
  const auto & curr = history.back();

  const double dt = curr.timestamp - prev.timestamp;
  constexpr double kMinTimeDelta = 0.01;  // 10ms minimum to avoid division issues
  if (dt < kMinTimeDelta) {
    return std::nullopt;
  }

  const double dx = curr.position.x - prev.position.x;
  const double dy = curr.position.y - prev.position.y;
  const double dz = curr.position.z - prev.position.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
}

std::vector<std::pair<std::vector<Eigen::Vector2d>, Intent>> TrajectoryPredictor::getAllLaneletPaths(
  const lanelet::ConstLanelet & start_lanelet, int max_depth) const
{
  std::vector<std::pair<std::vector<Eigen::Vector2d>, Intent>> all_paths;

  auto routing_graph = lanelet_handler_->getRoutingGraph();
  if (!routing_graph) {
    return all_paths;
  }

  // Cache start lanelet centerline (accessed multiple times)
  const auto & start_centerline = start_lanelet.centerline();
  if (start_centerline.empty()) {
    return all_paths;
  }

  // Get all possible following lanelets from the start
  auto following = routing_graph->following(start_lanelet);

  if (following.empty()) {
    // No following lanelets - just return path from start lanelet
    std::vector<Eigen::Vector2d> path;
    path.reserve(start_centerline.size());
    for (const auto & pt : start_centerline) {
      path.emplace_back(pt.x(), pt.y());
    }
    all_paths.emplace_back(std::move(path), Intent::CONTINUE_STRAIGHT);
    return all_paths;
  }

  // Pre-cache start lanelet points for reuse across all paths
  std::vector<Eigen::Vector2d> start_path_points;
  start_path_points.reserve(start_centerline.size());
  for (const auto & pt : start_centerline) {
    start_path_points.emplace_back(pt.x(), pt.y());
  }

  const auto start_end = start_centerline.back();
  all_paths.reserve(following.size());

  // Lateral displacement threshold for turn detection
  constexpr double kTurnThreshold = 1.0;  // meters

  // For each possible following lanelet, build a complete path
  for (const auto & next_lanelet : following) {
    std::vector<Eigen::Vector2d> path = start_path_points;  // Copy cached start points

    // Determine intent based on lateral displacement
    const auto & next_centerline = next_lanelet.centerline();
    Intent intent = Intent::CONTINUE_STRAIGHT;
    if (!next_centerline.empty()) {
      const auto next_start = next_centerline.front();
      const double lateral_diff = next_start.y() - start_end.y();
      if (lateral_diff > kTurnThreshold) {
        intent = Intent::TURN_LEFT;
      } else if (lateral_diff < -kTurnThreshold) {
        intent = Intent::TURN_RIGHT;
      }
    }

    // Continue building path following successive lanelets
    lanelet::ConstLanelet current = next_lanelet;
    for (int depth = 0; depth < max_depth; ++depth) {
      const auto & current_centerline = current.centerline();
      for (const auto & pt : current_centerline) {
        path.emplace_back(pt.x(), pt.y());
      }
      auto next_following = routing_graph->following(current);
      if (next_following.empty()) {
        break;
      }
      current = next_following.front();
    }

    if (path.size() >= 2) {
      all_paths.emplace_back(std::move(path), intent);
    }
  }

  return all_paths;
}

}  // namespace prediction
