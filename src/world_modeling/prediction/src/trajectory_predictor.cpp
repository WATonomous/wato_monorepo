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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace prediction
{

namespace
{
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
  std::vector<geometry_msgs::msg::PoseStamped> && poses,
  double /*time_step*/,
  Intent intent = Intent::CONTINUE_STRAIGHT)
{
  TrajectoryHypothesis hypothesis;
  hypothesis.intent = intent;
  hypothesis.probability = 0.0;
  hypothesis.poses = std::move(poses);
  return hypothesis;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
}  // namespace

TrajectoryPredictor::TrajectoryPredictor(
  rclcpp_lifecycle::LifecycleNode * node,
  double prediction_horizon,
  double time_step,
  const VehicleParams & vehicle_params,
  const PedestrianParams & pedestrian_params,
  const CyclistParams & cyclist_params,
  const TrajectoryPredictorConfig & config)
: node_(node)
, prediction_horizon_(prediction_horizon)
, time_step_(time_step)
, vehicle_params_(vehicle_params)
, pedestrian_params_(pedestrian_params)
, cyclist_params_(cyclist_params)
, config_(config)
{
  bicycle_model_ = std::make_unique<BicycleModel>(config_.bicycle_config);
  constant_velocity_model_ = std::make_unique<ConstantVelocityModel>(config_.cv_config);

  RCLCPP_INFO(node_->get_logger(), "TrajectoryPredictor initialized");
}

double TrajectoryPredictor::computeStopProbability(double speed) const
{
  return 1.0 / (1.0 + std::exp(config_.stop_sigmoid_steepness * (speed - config_.stop_sigmoid_midpoint)));
}

void TrajectoryPredictor::setLaneletAhead(const lanelet_msgs::msg::LaneletAhead::SharedPtr & msg)
{
  lanelet_cache_ = *msg;
  lanelet_id_to_index_.clear();
  for (size_t i = 0; i < lanelet_cache_.lanelets.size(); ++i) {
    lanelet_id_to_index_[lanelet_cache_.lanelets[i].id] = i;
  }
}

void TrajectoryPredictor::setLaneletQueryFunction(LaneletQueryFn fn)
{
  lanelet_query_fn_ = std::move(fn);
}

std::optional<TrajectoryPredictor::VehicleLaneletEntry> TrajectoryPredictor::queryVehicleLanelets(
  const std::string & vehicle_id, const geometry_msgs::msg::Point & position, double heading_rad)
{
  {
    std::lock_guard<std::mutex> lock(vehicle_cache_mutex_);
    auto it = vehicle_lanelet_cache_.find(vehicle_id);
    if (it != vehicle_lanelet_cache_.end()) {
      double dx = position.x - it->second.last_x;
      double dy = position.y - it->second.last_y;
      if (dx * dx + dy * dy < config_.vehicle_cache_invalidation_dist * config_.vehicle_cache_invalidation_dist) {
        return it->second;  // Return a copy — caller owns it
      }
    }
  }

  // Cache miss or stale — fire async request (non-blocking).
  // Results arrive via updateVehicleLaneletCache() and will be
  // available on the next prediction cycle.
  if (lanelet_query_fn_) {
    lanelet_query_fn_(vehicle_id, position, heading_rad);
  }
  return std::nullopt;
}

void TrajectoryPredictor::updateVehicleLaneletCache(
  const std::string & vehicle_id, const lanelet_msgs::msg::LaneletAhead & data, double x, double y)
{
  VehicleLaneletEntry entry;
  entry.lanelet_ahead = data;
  for (size_t i = 0; i < data.lanelets.size(); ++i) {
    entry.id_to_index[data.lanelets[i].id] = i;
  }
  entry.last_x = x;
  entry.last_y = y;
  entry.last_update = node_->get_clock()->now();

  std::lock_guard<std::mutex> lock(vehicle_cache_mutex_);
  vehicle_lanelet_cache_[vehicle_id] = std::move(entry);
}

double TrajectoryPredictor::estimateSpeed(const vision_msgs::msg::Detection3D & detection)
{
  rclcpp::Time now = node_->get_clock()->now();
  double x = detection.bbox.center.position.x;
  double y = detection.bbox.center.position.y;

  auto it = position_history_.find(detection.id);

  // First observation — no displacement history yet.
  // Return -1 sentinel: generators will skip stop/moving bias entirely,
  // producing uniform priors so the confidence smoother has no bad data.
  if (it == position_history_.end()) {
    position_history_[detection.id] = {x, y, 0.0, -1.0, now};
    return -1.0;
  }

  double dt = (now - it->second.stamp).seconds();
  double raw_speed = 0.0;

  if (dt > config_.min_dt_for_speed) {
    // Displacement-based speed — works for any dt, including stale tracks.
    double dx = x - it->second.x;
    double dy = y - it->second.y;
    raw_speed = std::sqrt(dx * dx + dy * dy) / dt;
    double max_speed =
      (detection.bbox.size.x > config_.vehicle_min_length) ? config_.max_vehicle_speed : config_.max_pedestrian_speed;
    raw_speed = std::clamp(raw_speed, 0.0, max_speed);
  } else {
    // Sub-centisecond dt: displacement too small to measure, carry forward.
    raw_speed = it->second.smoothed_speed;
  }

  // Second frame (first real measurement): previous smoothed_speed is the -1
  // sentinel.  Use raw_speed directly — don't EMA-blend against a meaningless
  // initial value.  After this, normal EMA applies.
  double smoothed;
  if (it->second.smoothed_speed < 0.0) {
    smoothed = raw_speed;
  } else {
    smoothed = config_.speed_ema_alpha * raw_speed + (1.0 - config_.speed_ema_alpha) * it->second.smoothed_speed;
  }

  position_history_[detection.id] = {x, y, raw_speed, smoothed, now};

  RCLCPP_DEBUG(
    node_->get_logger(),
    "Speed [%s]: raw=%.2f smoothed=%.2f dt=%.3f stop_prob=%.3f",
    detection.id.c_str(),
    raw_speed,
    smoothed,
    dt,
    (smoothed >= 0.0) ? computeStopProbability(smoothed) : -1.0);

  return smoothed;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::buildCvFallback(
  const KinematicState & state, double velocity, const char * reason)
{
  auto poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);
  auto hypothesis = buildHypothesis(std::move(poses), time_step_, Intent::CONTINUE_STRAIGHT);
  hypothesis.probability = 1.0;

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    5000,
    "Cyclist prediction: constant-velocity (%s), velocity=%.2f m/s",
    reason,
    velocity);

  return {std::move(hypothesis)};
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateHypotheses(
  const vision_msgs::msg::Detection3D & detection, double timestamp)
{
  std::optional<double> velocity =
    updateHistoryAndComputeVelocity(detection.id, detection.bbox.center.position, timestamp);

  // Map detected class to object type
  ObjectType obj_type = ObjectType::UNKNOWN;
  if (!detection.results.empty()) {
    const std::string & class_id = detection.results[0].hypothesis.class_id;
    if (class_id == "car" || class_id == "truck" || class_id == "bus" || class_id == "vehicle") {
      return generateVehicleHypotheses(detection, velocity);
    } else if (class_id == "pedestrian") {
      return generatePedestrianHypotheses(detection, velocity);
    } else if (class_id == "bicycle") {
      return generateCyclistHypotheses(detection, velocity);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unknown object type, skipping prediction");
      return {};
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "Detection has no classification results, skipping prediction");
    return {};
  }
}

ObjectType TrajectoryPredictor::classifyObjectType(const vision_msgs::msg::Detection3D & detection) const
{
  double length = detection.bbox.size.x;
  double height = detection.bbox.size.z;

  if (length > config_.vehicle_min_length) {
    return ObjectType::VEHICLE;
  } else if (length < config_.pedestrian_max_length && height > config_.pedestrian_max_length) {
    return ObjectType::PEDESTRIAN;
  } else if (length >= config_.pedestrian_max_length && length <= config_.cyclist_max_length) {
    return ObjectType::CYCLIST;
  }

  return ObjectType::VEHICLE;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateVehicleHypotheses(
  const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity)
{
  // Use computed velocity if available, otherwise default to typical vehicle
  // speed
  double v = velocity.value_or(vehicle_params_.default_speed);
  v = std::clamp(v, 0.0, vehicle_params_.max_speed);

  std::vector<TrajectoryHypothesis> hypotheses;

  // Try lanelet-aware prediction first
  if (lanelet_query_fn_) {
    double heading = extractYaw(detection.bbox.center.orientation);
    auto entry = queryVehicleLanelets(detection.id, detection.bbox.center.position, heading);
    if (entry && !entry->lanelet_ahead.lanelets.empty()) {
      LaneletContext ctx{entry->lanelet_ahead, entry->id_to_index};
      hypotheses = generateLaneletVehicleHypotheses(detection, v, ctx);
    }
  }

  // Fall back to geometric if lanelet prediction didn't produce results
  if (hypotheses.empty()) {
    hypotheses = generateGeometricVehicleHypotheses(detection, v);
  }

  return hypotheses;
}

// =============================================================================
// Lanelet-aware vehicle hypothesis generation
// =============================================================================

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateLaneletVehicleHypotheses(
  const vision_msgs::msg::Detection3D & detection, double speed, const LaneletContext & ctx)
{
  std::vector<TrajectoryHypothesis> hypotheses;

  rclcpp::Time current_time = node_->get_clock()->now();
  std::string frame_id = "map";

  KinematicState initial_state;
  initial_state.x = detection.bbox.center.position.x;
  initial_state.y = detection.bbox.center.position.y;
  initial_state.theta = extractYaw(detection.bbox.center.orientation);
  initial_state.v = std::max(0.0, speed);  // clamp sentinel -1 to 0 for kinematics
  initial_state.a = 0.0;
  initial_state.delta = 0.0;

  // Find which lanelet this vehicle is on
  LaneletMatch match = findVehicleLanelet(initial_state, ctx);
  if (!match.lanelet) {
    return hypotheses;  // No match, caller falls back to geometric
  }

  const auto & current_ll = *match.lanelet;
  double required_distance = initial_state.v * prediction_horizon_ + config_.required_distance_buffer;

  // =========================================================================
  // Curvature-aware scoring context
  // =========================================================================
  double road_curvature = computeCurvatureAtIndex(current_ll.centerline, match.closest_centerline_idx);

  double curvature_heading_tolerance =
    std::min(road_curvature * config_.curvature_heading_tolerance_scale, config_.curvature_heading_tolerance_max);
  double curvature_lateral_tolerance = std::min(
    speed * speed * road_curvature / config_.curvature_lateral_tolerance_denom,
    config_.curvature_lateral_tolerance_max);

  double adjusted_heading_diff = std::max(0.0, match.heading_diff - curvature_heading_tolerance);
  double adjusted_lateral_offset = (std::abs(match.lateral_offset) > curvature_lateral_tolerance)
                                     ? match.lateral_offset
                                     : match.lateral_offset * config_.curve_offset_discount;

  // =========================================================================
  // Stop probability
  // =========================================================================
  const double stop_prob = (speed >= 0.0) ? computeStopProbability(speed) : 0.0;

  if (stop_prob > config_.stop_probability_threshold) {
    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::STOP;
    hyp.probability = stop_prob;

    for (double t = time_step_; t <= prediction_horizon_; t += time_step_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = frame_id;
      pose_stamped.header.stamp = current_time + rclcpp::Duration::from_seconds(t);
      pose_stamped.pose.position.x = initial_state.x;
      pose_stamped.pose.position.y = initial_state.y;
      pose_stamped.pose.position.z = 0.0;
      pose_stamped.pose.orientation = detection.bbox.center.orientation;
      hyp.poses.push_back(pose_stamped);
    }

    hyp.lanelet_ids.push_back(current_ll.id);
    hypotheses.push_back(hyp);
  }

  // =========================================================================
  // Maneuver inertia — retrieve previous dominant intent
  // =========================================================================
  Intent prev_intent = Intent::UNKNOWN;
  auto prev_it = previous_intent_.find(detection.id);
  if (prev_it != previous_intent_.end()) {
    prev_intent = prev_it->second.intent;
  }

  // =========================================================================
  // Lateral velocity for lane-change evidence
  // =========================================================================
  double lateral_vel = estimateLateralVelocity(detection.id, match.lateral_offset);

  const double abs_adjusted_offset = std::abs(adjusted_lateral_offset);
  const double lane_change_position_evidence = std::clamp(
    (abs_adjusted_offset - config_.lane_change_offset_threshold) / config_.lane_change_offset_scale, 0.0, 1.0);

  // --- Hypothesis: Follow current lane (continue straight / follow road) ---
  {
    auto path = extractForwardPath(current_ll, match.closest_centerline_idx, required_distance, ctx);
    if (path.size() >= 2) {
      TrajectoryHypothesis hyp;
      hyp.header.stamp = current_time;
      hyp.header.frame_id = frame_id;

      // Determine intent based on lanelet geometry
      if (current_ll.is_intersection) {
        double entry_heading = std::atan2(path[1].y() - path[0].y(), path[1].x() - path[0].x());
        size_t last = path.size() - 1;
        double exit_heading = std::atan2(path[last].y() - path[last - 1].y(), path[last].x() - path[last - 1].x());
        double heading_change = normalizeAngle(exit_heading - entry_heading);
        if (heading_change > config_.heading_change_threshold) {
          hyp.intent = Intent::TURN_LEFT;
        } else if (heading_change < -config_.heading_change_threshold) {
          hyp.intent = Intent::TURN_RIGHT;
        } else {
          hyp.intent = Intent::CONTINUE_STRAIGHT;
        }
      } else {
        hyp.intent = Intent::CONTINUE_STRAIGHT;
      }

      hyp.probability =
        computeGeometricScore(adjusted_heading_diff, adjusted_lateral_offset, config_.lane_follow_prior);

      hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, path, prediction_horizon_, time_step_, current_time, frame_id);

      if (!hyp.poses.empty()) {
        double smoothness = computeTrajectorySmoothness(hyp.poses);
        double smoothness_factor = std::exp(-smoothness * config_.smoothness_penalty_exponent);
        hyp.probability *= (config_.smoothness_base_weight + config_.smoothness_base_weight * smoothness_factor);

        if (prev_intent != Intent::UNKNOWN && prev_intent == hyp.intent) {
          hyp.probability *= config_.maneuver_inertia_boost;
        }

        hyp.lanelet_ids = extractForwardPathLaneletIds(current_ll, required_distance, ctx);

        hypotheses.push_back(hyp);
      }
    }
  }

  // --- Hypothesis: Follow each successor lanelet (for intersections with
  // multiple exits) ---
  if (current_ll.successor_ids.size() > 1) {
    for (int64_t succ_id : current_ll.successor_ids) {
      const auto * succ_ll = findLaneletById(succ_id, ctx);
      if (!succ_ll || succ_ll->centerline.size() < 2) continue;

      // Build path: remaining current centerline + successor centerline
      std::vector<Eigen::Vector2d> path;
      for (size_t i = match.closest_centerline_idx; i < current_ll.centerline.size(); ++i) {
        path.emplace_back(current_ll.centerline[i].x, current_ll.centerline[i].y);
      }
      for (const auto & pt : succ_ll->centerline) {
        path.emplace_back(pt.x, pt.y);
      }
      if (computePathLength(path) < required_distance) {
        for (int64_t succ_succ_id : succ_ll->successor_ids) {
          const auto * ss = findLaneletById(succ_succ_id, ctx);
          if (ss) {
            for (const auto & pt : ss->centerline) {
              path.emplace_back(pt.x, pt.y);
            }
            if (computePathLength(path) >= required_distance) break;
          }
        }
      }

      if (path.size() < 2) continue;

      TrajectoryHypothesis hyp;
      hyp.header.stamp = current_time;
      hyp.header.frame_id = frame_id;

      double entry_heading = std::atan2(path[1].y() - path[0].y(), path[1].x() - path[0].x());
      size_t last = path.size() - 1;
      double exit_heading = std::atan2(path[last].y() - path[last - 1].y(), path[last].x() - path[last - 1].x());
      double heading_change = normalizeAngle(exit_heading - entry_heading);

      if (heading_change > config_.heading_change_threshold) {
        hyp.intent = Intent::TURN_LEFT;
      } else if (heading_change < -config_.heading_change_threshold) {
        hyp.intent = Intent::TURN_RIGHT;
      } else {
        hyp.intent = Intent::CONTINUE_STRAIGHT;
      }

      double maneuver_prior = (std::abs(heading_change) > config_.heading_change_threshold)
                                ? config_.turn_maneuver_prior
                                : config_.continue_maneuver_prior;
      hyp.probability = computeGeometricScore(adjusted_heading_diff, adjusted_lateral_offset, maneuver_prior);

      hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, path, prediction_horizon_, time_step_, current_time, frame_id);

      if (!hyp.poses.empty()) {
        double smoothness = computeTrajectorySmoothness(hyp.poses);
        double smoothness_factor = std::exp(-smoothness * config_.smoothness_penalty_exponent);
        hyp.probability *= (config_.smoothness_base_weight + config_.smoothness_base_weight * smoothness_factor);

        if (prev_intent != Intent::UNKNOWN && prev_intent == hyp.intent) {
          hyp.probability *= config_.maneuver_inertia_boost;
        }

        hyp.lanelet_ids.push_back(current_ll.id);
        hyp.lanelet_ids.push_back(succ_id);
        for (int64_t succ_succ_id : succ_ll->successor_ids) {
          const auto * ss = findLaneletById(succ_succ_id, ctx);
          if (ss) {
            hyp.lanelet_ids.push_back(succ_succ_id);
            break;
          }
        }

        hypotheses.push_back(hyp);
      }
    }
  }

  // --- Hypothesis: Lane change left ---
  if (current_ll.can_change_left && current_ll.left_lane_id > 0) {
    auto path =
      extractLaneChangePath(current_ll, match.closest_centerline_idx, current_ll.left_lane_id, required_distance, ctx);
    if (path.size() >= 2) {
      TrajectoryHypothesis hyp;
      hyp.header.stamp = current_time;
      hyp.header.frame_id = frame_id;
      hyp.intent = Intent::LANE_CHANGE_LEFT;

      double lc_prior = config_.lane_change_base_prior;
      if (lateral_vel > config_.lateral_velocity_threshold) {
        lc_prior = config_.lane_change_active_prior;
      }

      if (current_ll.is_intersection) {
        lc_prior *= config_.intersection_suppression_factor;
      }

      if (road_curvature > config_.curvature_threshold) {
        lc_prior *= std::max(
          config_.curvature_suppression_minimum, 1.0 - road_curvature * config_.curvature_suppression_multiplier);
      }

      lc_prior *= (config_.position_evidence_base + config_.position_evidence_scale * lane_change_position_evidence);
      if (adjusted_lateral_offset > config_.lateral_offset_threshold) {
        lc_prior *= config_.lateral_offset_boost;
      } else if (adjusted_lateral_offset < -config_.lateral_offset_threshold) {
        lc_prior *= config_.lateral_offset_suppress;
      }

      hyp.probability = computeGeometricScore(
        adjusted_heading_diff * config_.heading_weight_for_lane_change, adjusted_lateral_offset, lc_prior);

      hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, path, prediction_horizon_, time_step_, current_time, frame_id);

      if (!hyp.poses.empty()) {
        double smoothness = computeTrajectorySmoothness(hyp.poses);
        double smoothness_factor = std::exp(-smoothness * config_.smoothness_penalty_exponent);
        hyp.probability *= (config_.smoothness_base_weight + config_.smoothness_base_weight * smoothness_factor);

        if (prev_intent == Intent::LANE_CHANGE_LEFT) {
          hyp.probability *= config_.lane_change_completion_boost;
        } else if (prev_intent == Intent::LANE_CHANGE_RIGHT) {
          hyp.probability *= config_.lane_change_direction_switch_suppress;
        }

        hyp.lanelet_ids = extractLaneChangePathLaneletIds(current_ll, current_ll.left_lane_id, required_distance, ctx);

        hypotheses.push_back(hyp);
      }
    }
  }

  // --- Hypothesis: Lane change right ---
  if (current_ll.can_change_right && current_ll.right_lane_id > 0) {
    auto path =
      extractLaneChangePath(current_ll, match.closest_centerline_idx, current_ll.right_lane_id, required_distance, ctx);
    if (path.size() >= 2) {
      TrajectoryHypothesis hyp;
      hyp.header.stamp = current_time;
      hyp.header.frame_id = frame_id;
      hyp.intent = Intent::LANE_CHANGE_RIGHT;

      double lc_prior = config_.lane_change_base_prior;
      if (lateral_vel < -config_.lateral_velocity_threshold) {
        lc_prior = config_.lane_change_active_prior;
      }

      if (current_ll.is_intersection) {
        lc_prior *= config_.intersection_suppression_factor;
      }

      if (road_curvature > config_.curvature_threshold) {
        lc_prior *= std::max(
          config_.curvature_suppression_minimum, 1.0 - road_curvature * config_.curvature_suppression_multiplier);
      }

      lc_prior *= (config_.position_evidence_base + config_.position_evidence_scale * lane_change_position_evidence);
      if (adjusted_lateral_offset < -config_.lateral_offset_threshold) {
        lc_prior *= config_.lateral_offset_boost;
      } else if (adjusted_lateral_offset > config_.lateral_offset_threshold) {
        lc_prior *= config_.lateral_offset_suppress;
      }

      hyp.probability = computeGeometricScore(
        adjusted_heading_diff * config_.heading_weight_for_lane_change, adjusted_lateral_offset, lc_prior);

      hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, path, prediction_horizon_, time_step_, current_time, frame_id);

      if (!hyp.poses.empty()) {
        double smoothness = computeTrajectorySmoothness(hyp.poses);
        double smoothness_factor = std::exp(-smoothness * config_.smoothness_penalty_exponent);
        hyp.probability *= (config_.smoothness_base_weight + config_.smoothness_base_weight * smoothness_factor);

        if (prev_intent == Intent::LANE_CHANGE_RIGHT) {
          hyp.probability *= config_.lane_change_completion_boost;
        } else if (prev_intent == Intent::LANE_CHANGE_LEFT) {
          hyp.probability *= config_.lane_change_direction_switch_suppress;
        }

        hyp.lanelet_ids = extractLaneChangePathLaneletIds(current_ll, current_ll.right_lane_id, required_distance, ctx);

        hypotheses.push_back(hyp);
      }
    }
  }

  // Scale moving hypotheses by (1 - stop_prob) so STOP and moving
  // hypotheses are calibrated by the same sigmoid.
  for (auto & h : hypotheses) {
    if (h.intent != Intent::STOP) {
      h.probability *= (1.0 - stop_prob);
    }
  }

  // Normalize probabilities
  if (!hypotheses.empty()) {
    double total = 0.0;
    for (const auto & h : hypotheses) total += h.probability;
    if (total > 1e-6) {
      for (auto & h : hypotheses) h.probability /= total;
    } else {
      double uniform = 1.0 / hypotheses.size();
      for (auto & h : hypotheses) h.probability = uniform;
    }

    // Record the dominant intent for maneuver inertia on next cycle
    double best_prob = 0.0;
    Intent best_intent = Intent::UNKNOWN;
    for (const auto & h : hypotheses) {
      if (h.probability > best_prob) {
        best_prob = h.probability;
        best_intent = h.intent;
      }
    }
    previous_intent_[detection.id] = {best_intent, node_->get_clock()->now()};
  }

  RCLCPP_DEBUG(
    node_->get_logger(),
    "Generated %zu lanelet-based vehicle hypotheses (lanelet %ld)",
    hypotheses.size(),
    current_ll.id);

  return hypotheses;
}

// =============================================================================
// Lanelet helper implementations
// =============================================================================

TrajectoryPredictor::LaneletMatch TrajectoryPredictor::findVehicleLanelet(
  const KinematicState & state, const LaneletContext & ctx) const
{
  LaneletMatch best_match{nullptr, 0, std::numeric_limits<double>::max(), M_PI};
  double best_score = std::numeric_limits<double>::max();

  for (const auto & ll : ctx.data.lanelets) {
    if (ll.centerline.size() < 2) continue;

    for (size_t i = 0; i < ll.centerline.size(); ++i) {
      double dx = ll.centerline[i].x - state.x;
      double dy = ll.centerline[i].y - state.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist > config_.lanelet_match_max_distance) continue;

      // Compute heading at this centerline point
      double cl_heading = computeHeadingAtIndex(ll.centerline, i);
      double heading_diff = std::abs(normalizeAngle(state.theta - cl_heading));

      // Combined score: distance + heading penalty
      double score = dist + config_.heading_penalty_weight * heading_diff;

      if (score < best_score) {
        best_score = score;

        // Compute signed lateral offset (positive = left of centerline)
        double cl_dx = std::cos(cl_heading);
        double cl_dy = std::sin(cl_heading);
        double to_vehicle_x = state.x - ll.centerline[i].x;
        double to_vehicle_y = state.y - ll.centerline[i].y;
        double lateral = -cl_dx * to_vehicle_y + cl_dy * to_vehicle_x;

        best_match.lanelet = &ll;
        best_match.closest_centerline_idx = i;
        best_match.lateral_offset = lateral;
        best_match.heading_diff = heading_diff;
      }
    }
  }

  // Reject if heading difference is too large (vehicle going wrong direction on
  // this lanelet)
  if (best_match.lanelet && best_match.heading_diff > config_.heading_rejection_threshold) {
    best_match.lanelet = nullptr;
  }

  return best_match;
}

std::vector<Eigen::Vector2d> TrajectoryPredictor::extractForwardPath(
  const lanelet_msgs::msg::Lanelet & lanelet,
  size_t start_idx,
  double required_distance,
  const LaneletContext & ctx) const
{
  std::vector<Eigen::Vector2d> path;

  // Add remaining centerline points from current lanelet
  for (size_t i = start_idx; i < lanelet.centerline.size(); ++i) {
    path.emplace_back(lanelet.centerline[i].x, lanelet.centerline[i].y);
  }

  // Extend through successors if needed
  double accumulated = computePathLength(path);
  std::vector<int64_t> to_visit = {lanelet.successor_ids.begin(), lanelet.successor_ids.end()};
  size_t visit_idx = 0;

  while (accumulated < required_distance && visit_idx < to_visit.size()) {
    const auto * succ = findLaneletById(to_visit[visit_idx], ctx);
    ++visit_idx;
    if (!succ) continue;

    for (const auto & pt : succ->centerline) {
      path.emplace_back(pt.x, pt.y);
    }
    accumulated = computePathLength(path);

    // Add this successor's successors (BFS, but only first to keep it simple)
    if (accumulated < required_distance && !succ->successor_ids.empty()) {
      to_visit.push_back(succ->successor_ids[0]);
    }
  }

  return path;
}

std::vector<Eigen::Vector2d> TrajectoryPredictor::extractLaneChangePath(
  const lanelet_msgs::msg::Lanelet & current_lanelet,
  size_t start_idx,
  int64_t target_lane_id,
  double required_distance,
  const LaneletContext & ctx) const
{
  const auto * target_ll = findLaneletById(target_lane_id, ctx);
  if (!target_ll || target_ll->centerline.size() < 2) {
    return {};
  }

  std::vector<Eigen::Vector2d> path;

  // Use a portion of the current lane, then smoothly transition to target lane
  // Take ~1/3 of remaining current centerline, then transition
  size_t remaining = current_lanelet.centerline.size() - start_idx;
  size_t transition_start = start_idx + std::min(remaining, remaining / 3 + 1);

  // Current lane portion
  for (size_t i = start_idx; i < transition_start; ++i) {
    path.emplace_back(current_lanelet.centerline[i].x, current_lanelet.centerline[i].y);
  }

  // Find closest point on target lane to the transition point
  if (path.empty()) return {};
  Eigen::Vector2d transition_pt = path.back();
  double min_dist = std::numeric_limits<double>::max();
  size_t target_start = 0;

  for (size_t i = 0; i < target_ll->centerline.size(); ++i) {
    double dx = target_ll->centerline[i].x - transition_pt.x();
    double dy = target_ll->centerline[i].y - transition_pt.y();
    double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      target_start = i;
    }
  }

  // Smooth transition: interpolate between current and target for a few points
  size_t blend_count = static_cast<size_t>(config_.lane_change_blend_count);
  if (target_start + blend_count < target_ll->centerline.size() && !path.empty()) {
    Eigen::Vector2d from = path.back();
    for (size_t i = 0; i < blend_count; ++i) {
      double t = static_cast<double>(i + 1) / static_cast<double>(blend_count);
      size_t tgt_idx = target_start + i;
      if (tgt_idx >= target_ll->centerline.size()) break;
      double x = from.x() * (1.0 - t) + target_ll->centerline[tgt_idx].x * t;
      double y = from.y() * (1.0 - t) + target_ll->centerline[tgt_idx].y * t;
      path.emplace_back(x, y);
    }
    target_start += blend_count;
  }

  // Continue along target lane
  for (size_t i = target_start; i < target_ll->centerline.size(); ++i) {
    path.emplace_back(target_ll->centerline[i].x, target_ll->centerline[i].y);
  }

  // Extend through target's successors if needed
  double accumulated = computePathLength(path);
  if (accumulated < required_distance && !target_ll->successor_ids.empty()) {
    const auto * succ = findLaneletById(target_ll->successor_ids[0], ctx);
    if (succ) {
      for (const auto & pt : succ->centerline) {
        path.emplace_back(pt.x, pt.y);
      }
    }
  }

  return path;
}

const lanelet_msgs::msg::Lanelet * TrajectoryPredictor::findLaneletById(int64_t id, const LaneletContext & ctx) const
{
  auto it = ctx.id_to_index.find(id);
  if (it != ctx.id_to_index.end() && it->second < ctx.data.lanelets.size()) {
    return &ctx.data.lanelets[it->second];
  }
  return nullptr;
}

std::vector<int64_t> TrajectoryPredictor::extractForwardPathLaneletIds(
  const lanelet_msgs::msg::Lanelet & lanelet, double required_distance, const LaneletContext & ctx) const
{
  std::vector<int64_t> lanelet_ids;
  lanelet_ids.push_back(lanelet.id);

  // Extend through successors if needed
  std::vector<int64_t> to_visit(lanelet.successor_ids.begin(), lanelet.successor_ids.end());
  size_t visit_idx = 0;
  double accumulated = 0.0;

  for (size_t i = 1; i < lanelet.centerline.size(); ++i) {
    double dx = lanelet.centerline[i].x - lanelet.centerline[i - 1].x;
    double dy = lanelet.centerline[i].y - lanelet.centerline[i - 1].y;
    accumulated += std::sqrt(dx * dx + dy * dy);
  }

  while (accumulated < required_distance && visit_idx < to_visit.size()) {
    const auto * succ = findLaneletById(to_visit[visit_idx], ctx);
    ++visit_idx;
    if (!succ) continue;

    lanelet_ids.push_back(succ->id);
    for (size_t i = 1; i < succ->centerline.size(); ++i) {
      double dx = succ->centerline[i].x - succ->centerline[i - 1].x;
      double dy = succ->centerline[i].y - succ->centerline[i - 1].y;
      accumulated += std::sqrt(dx * dx + dy * dy);
    }

    // Add this successor's successors
    if (accumulated < required_distance && !succ->successor_ids.empty()) {
      to_visit.push_back(succ->successor_ids[0]);
    }
  }

  return lanelet_ids;
}

std::vector<int64_t> TrajectoryPredictor::extractLaneChangePathLaneletIds(
  const lanelet_msgs::msg::Lanelet & current_lanelet,
  int64_t target_lane_id,
  double required_distance,
  const LaneletContext & ctx) const
{
  std::vector<int64_t> lanelet_ids;
  lanelet_ids.push_back(current_lanelet.id);

  const auto * target_ll = findLaneletById(target_lane_id, ctx);
  if (!target_ll) return lanelet_ids;

  lanelet_ids.push_back(target_ll->id);

  // Add target's successors if needed
  double accumulated = 0.0;
  for (size_t i = 1; i < target_ll->centerline.size(); ++i) {
    double dx = target_ll->centerline[i].x - target_ll->centerline[i - 1].x;
    double dy = target_ll->centerline[i].y - target_ll->centerline[i - 1].y;
    accumulated += std::sqrt(dx * dx + dy * dy);
  }

  if (accumulated < required_distance && !target_ll->successor_ids.empty()) {
    const auto * succ = findLaneletById(target_ll->successor_ids[0], ctx);
    if (succ) {
      lanelet_ids.push_back(succ->id);
    }
  }

  return lanelet_ids;
}

double TrajectoryPredictor::computePathLength(const std::vector<Eigen::Vector2d> & path) const
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}

double TrajectoryPredictor::computeHeadingAtIndex(
  const std::vector<geometry_msgs::msg::Point> & centerline, size_t idx) const
{
  if (centerline.size() < 2) return 0.0;

  size_t i0, i1;
  if (idx + 1 < centerline.size()) {
    i0 = idx;
    i1 = idx + 1;
  } else {
    i0 = idx - 1;
    i1 = idx;
  }

  return std::atan2(centerline[i1].y - centerline[i0].y, centerline[i1].x - centerline[i0].x);
}

double TrajectoryPredictor::computeGeometricScore(
  double heading_diff, double lateral_offset, double maneuver_prior) const
{
  // Gaussian scoring: higher when heading aligns and vehicle is near centerline
  double heading_score = std::exp(-heading_diff * heading_diff / config_.heading_score_denominator);
  double lateral_score = std::exp(-lateral_offset * lateral_offset / config_.lateral_score_denominator);
  return heading_score * lateral_score * maneuver_prior;
}

double TrajectoryPredictor::computeCurvatureAtIndex(
  const std::vector<geometry_msgs::msg::Point> & centerline, size_t idx) const
{
  if (centerline.size() < 3) {
    return 0.0;
  }
  size_t i0 = (idx > 0) ? idx - 1 : 0;
  size_t i1 = idx;
  size_t i2 = (idx + 1 < centerline.size()) ? idx + 1 : centerline.size() - 1;
  if (i0 == i1 || i1 == i2) {
    return 0.0;
  }

  double ax = centerline[i1].x - centerline[i0].x;
  double ay = centerline[i1].y - centerline[i0].y;
  double bx = centerline[i2].x - centerline[i1].x;
  double by = centerline[i2].y - centerline[i1].y;

  double cross = std::abs(ax * by - ay * bx);
  double la = std::sqrt(ax * ax + ay * ay);
  double lb = std::sqrt(bx * bx + by * by);
  double lc = std::sqrt(
    (centerline[i2].x - centerline[i0].x) * (centerline[i2].x - centerline[i0].x) +
    (centerline[i2].y - centerline[i0].y) * (centerline[i2].y - centerline[i0].y));

  double denom = la * lb * lc;
  if (denom < 1e-9) {
    return 0.0;
  }
  return 2.0 * cross / denom;
}

double TrajectoryPredictor::computeTrajectorySmoothness(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses) const
{
  if (poses.size() < 3) {
    return 0.0;
  }
  double total_curvature = 0.0;
  size_t valid_count = 0;
  for (size_t i = 1; i + 1 < poses.size(); ++i) {
    double dx1 = poses[i].pose.position.x - poses[i - 1].pose.position.x;
    double dy1 = poses[i].pose.position.y - poses[i - 1].pose.position.y;
    double dx2 = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    double dy2 = poses[i + 1].pose.position.y - poses[i].pose.position.y;

    if ((dx1 * dx1 + dy1 * dy1) < 1e-8 || (dx2 * dx2 + dy2 * dy2) < 1e-8) {
      continue;
    }

    double h1 = std::atan2(dy1, dx1);
    double h2 = std::atan2(dy2, dx2);
    total_curvature += std::abs(normalizeAngle(h2 - h1));
    ++valid_count;
  }
  return (valid_count > 0) ? total_curvature / static_cast<double>(valid_count) : 0.0;
}

double TrajectoryPredictor::estimateLateralVelocity(const std::string & vehicle_id, double lateral_offset)
{
  rclcpp::Time now = node_->get_clock()->now();
  double lateral_vel = 0.0;

  auto it = lateral_offset_history_.find(vehicle_id);
  if (it != lateral_offset_history_.end()) {
    double dt = (now - it->second.stamp).seconds();
    if (dt > config_.lateral_velocity_min_dt && dt < config_.lateral_velocity_max_dt) {
      lateral_vel = (lateral_offset - it->second.lateral_offset) / dt;
    }
  }

  lateral_offset_history_[vehicle_id] = {lateral_offset, now};
  return lateral_vel;
}

void TrajectoryPredictor::pruneStaleCaches(const rclcpp::Time & now, double ttl_s)
{
  for (auto it = position_history_.begin(); it != position_history_.end();) {
    if ((now - it->second.stamp).seconds() > ttl_s) {
      it = position_history_.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = lateral_offset_history_.begin(); it != lateral_offset_history_.end();) {
    if ((now - it->second.stamp).seconds() > ttl_s) {
      it = lateral_offset_history_.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = previous_intent_.begin(); it != previous_intent_.end();) {
    if ((now - it->second.stamp).seconds() > ttl_s) {
      it = previous_intent_.erase(it);
    } else {
      ++it;
    }
  }

  {
    std::lock_guard<std::mutex> lock(vehicle_cache_mutex_);
    for (auto it = vehicle_lanelet_cache_.begin(); it != vehicle_lanelet_cache_.end();) {
      if ((now - it->second.last_update).seconds() > ttl_s) {
        it = vehicle_lanelet_cache_.erase(it);
      } else {
        ++it;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    const double now_sec = now.seconds();
    for (auto it = detection_history_.begin(); it != detection_history_.end();) {
      if (it->second.empty() || (now_sec - it->second.back().timestamp) > ttl_s) {
        it = detection_history_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

// =============================================================================
// Geometric fallback (original behavior, used when no lanelet data)
// =============================================================================

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateGeometricVehicleHypotheses(
  const vision_msgs::msg::Detection3D & detection, double speed)
{
  std::vector<TrajectoryHypothesis> hypotheses;

  rclcpp::Time current_time = node_->get_clock()->now();
  std::string frame_id = "map";

  KinematicState initial_state;
  initial_state.x = detection.bbox.center.position.x;
  initial_state.y = detection.bbox.center.position.y;
  initial_state.theta = extractYaw(detection.bbox.center.orientation);
  initial_state.v = std::max(0.0, speed);  // clamp sentinel -1 to 0 for kinematics
  initial_state.a = 0.0;
  initial_state.delta = 0.0;

  // Stop hypothesis — same sigmoid as lanelet-aware path.
  const double stop_prob = (speed >= 0.0) ? computeStopProbability(speed) : 0.0;

  if (stop_prob > config_.stop_probability_threshold) {
    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::STOP;
    hyp.probability = stop_prob;

    for (double t = time_step_; t <= prediction_horizon_; t += time_step_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = frame_id;
      pose_stamped.header.stamp = current_time + rclcpp::Duration::from_seconds(t);
      pose_stamped.pose.position.x = initial_state.x;
      pose_stamped.pose.position.y = initial_state.y;
      pose_stamped.pose.position.z = detection.bbox.center.position.z;
      pose_stamped.pose.orientation = detection.bbox.center.orientation;
      hyp.poses.push_back(pose_stamped);
    }

    hypotheses.push_back(hyp);
  }

  // Continue Straight
  {
    std::vector<Eigen::Vector2d> straight_path;
    for (double d = 0.0; d <= initial_state.v * prediction_horizon_ + config_.required_distance_buffer;
         d += config_.geometric_path_sampling_interval)
    {
      straight_path.emplace_back(
        initial_state.x + d * std::cos(initial_state.theta), initial_state.y + d * std::sin(initial_state.theta));
    }

    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::CONTINUE_STRAIGHT;
    hyp.probability = config_.geometric_straight_probability;
    hyp.poses = bicycle_model_->generateTrajectory(
      initial_state, straight_path, prediction_horizon_, time_step_, current_time, frame_id);
    hypotheses.push_back(hyp);
  }

  // Turn Left
  {
    std::vector<Eigen::Vector2d> left_path;
    double turn_radius = config_.geometric_turn_radius;
    for (double angle = 0.0; angle <= M_PI / 2.0; angle += config_.geometric_turn_angle_step) {
      double path_x = initial_state.x + turn_radius * std::sin(angle) * std::cos(initial_state.theta) -
                      turn_radius * (1.0 - std::cos(angle)) * std::sin(initial_state.theta);
      double path_y = initial_state.y + turn_radius * std::sin(angle) * std::sin(initial_state.theta) +
                      turn_radius * (1.0 - std::cos(angle)) * std::cos(initial_state.theta);
      left_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::TURN_LEFT;
    hyp.probability = config_.geometric_turn_probability;
    hyp.poses = bicycle_model_->generateTrajectory(
      initial_state, left_path, prediction_horizon_, time_step_, current_time, frame_id);
    hypotheses.push_back(hyp);
  }

  // Turn Right
  {
    std::vector<Eigen::Vector2d> right_path;
    double turn_radius = config_.geometric_turn_radius;
    for (double angle = 0.0; angle <= M_PI / 2.0; angle += config_.geometric_turn_angle_step) {
      double path_x = initial_state.x + turn_radius * std::sin(angle) * std::cos(initial_state.theta) +
                      turn_radius * (1.0 - std::cos(angle)) * std::sin(initial_state.theta);
      double path_y = initial_state.y + turn_radius * std::sin(angle) * std::sin(initial_state.theta) -
                      turn_radius * (1.0 - std::cos(angle)) * std::cos(initial_state.theta);
      right_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::TURN_RIGHT;
    hyp.probability = config_.geometric_turn_probability;
    hyp.poses = bicycle_model_->generateTrajectory(
      initial_state, right_path, prediction_horizon_, time_step_, current_time, frame_id);
    hypotheses.push_back(hyp);
  }

  // Lane Change Left
  {
    std::vector<Eigen::Vector2d> lcl_path;
    double lane_width = config_.geometric_lane_width;
    double lc_distance = config_.geometric_lane_change_distance;
    for (double d = 0.0; d <= lc_distance; d += config_.geometric_path_sampling_interval) {
      double lateral_offset = lane_width * (d / lc_distance);
      double path_x =
        initial_state.x + d * std::cos(initial_state.theta) - lateral_offset * std::sin(initial_state.theta);
      double path_y =
        initial_state.y + d * std::sin(initial_state.theta) + lateral_offset * std::cos(initial_state.theta);
      lcl_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::LANE_CHANGE_LEFT;
    hyp.probability = config_.geometric_lane_change_probability;
    hyp.poses = bicycle_model_->generateTrajectory(
      initial_state, lcl_path, prediction_horizon_, time_step_, current_time, frame_id);
    hypotheses.push_back(hyp);
  }

  // Lane Change Right
  {
    std::vector<Eigen::Vector2d> lcr_path;
    double lane_width = config_.geometric_lane_width;
    double lc_distance = config_.geometric_lane_change_distance;
    for (double d = 0.0; d <= lc_distance; d += config_.geometric_path_sampling_interval) {
      double lateral_offset = lane_width * (d / lc_distance);
      double path_x =
        initial_state.x + d * std::cos(initial_state.theta) + lateral_offset * std::sin(initial_state.theta);
      double path_y =
        initial_state.y + d * std::sin(initial_state.theta) - lateral_offset * std::cos(initial_state.theta);
      lcr_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::LANE_CHANGE_RIGHT;
    hyp.probability = config_.geometric_lane_change_probability;
    hyp.poses = bicycle_model_->generateTrajectory(
      initial_state, lcr_path, prediction_horizon_, time_step_, current_time, frame_id);
    hypotheses.push_back(hyp);
  }

  // Scale moving hypotheses by (1 - stop_prob)
  for (auto & h : hypotheses) {
    if (h.intent != Intent::STOP) {
      h.probability *= (1.0 - stop_prob);
    }
  }

  // Normalize probabilities
  if (!hypotheses.empty()) {
    double total = 0.0;
    for (const auto & h : hypotheses) total += h.probability;
    if (total > 1e-6) {
      for (auto & h : hypotheses) h.probability /= total;
    } else {
      double uniform = 1.0 / hypotheses.size();
      for (auto & h : hypotheses) h.probability = uniform;
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "Generated %zu geometric vehicle hypotheses (no lanelet data)", hypotheses.size());

  return hypotheses;
}

// =============================================================================
// Pedestrian and Cyclist hypotheses
// =============================================================================

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generatePedestrianHypotheses(
  const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity)
{
  // Use computed velocity if available, otherwise default to walking speed
  double v = velocity.value_or(pedestrian_params_.default_speed);
  v = std::clamp(v, 0.0, pedestrian_params_.max_speed);

  auto state = stateFromDetection(detection.bbox.center.position, detection.bbox.center.orientation, v);

  auto poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Pedestrian prediction: velocity=%.2f m/s", v);

  return {buildHypothesis(std::move(poses), time_step_)};
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateCyclistHypotheses(
  const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity)
{
  const auto & pos = detection.bbox.center.position;
  const double default_speed = (cyclist_params_.min_speed + cyclist_params_.max_speed) * 0.5;
  double v = velocity.value_or(default_speed);
  v = std::clamp(v, cyclist_params_.min_speed, cyclist_params_.max_speed);

  auto state = stateFromDetection(pos, detection.bbox.center.orientation, v);

  if (lanelet_query_fn_) {
    double heading = extractYaw(detection.bbox.center.orientation);
    auto entry = queryVehicleLanelets(detection.id, pos, heading);
    if (entry && !entry->lanelet_ahead.lanelets.empty()) {
      LaneletContext ctx{entry->lanelet_ahead, entry->id_to_index};
      auto match = findVehicleLanelet(state, ctx);
      if (match.lanelet && match.lateral_offset <= cyclist_params_.lanelet_proximity_threshold) {
        auto all_paths = getAllLaneletPaths(*match.lanelet, cyclist_params_.max_lanelet_search_depth, ctx);

        const double kLaneletTotalConf = cyclist_params_.lanelet_confidence;
        const double kCvFallbackConf = cyclist_params_.cv_fallback_confidence;
        const double kStraightBoost = cyclist_params_.straight_boost;

        double total_weight = 0.0;
        for (const auto & [path, intent] : all_paths) {
          if (path.size() >= 2) {
            total_weight += (intent == Intent::CONTINUE_STRAIGHT) ? kStraightBoost : 1.0;
          }
        }

        if (total_weight > 0.0) {
          std::vector<TrajectoryHypothesis> hypotheses;
          hypotheses.reserve(all_paths.size() + 1);

          rclcpp::Time current_time = node_->get_clock()->now();
          for (auto & [path_points, intent] : all_paths) {
            if (path_points.size() < 2) continue;
            TrajectoryHypothesis hyp;
            hyp.intent = intent;
            hyp.poses = bicycle_model_->generateTrajectory(
              state, path_points, prediction_horizon_, time_step_, current_time, "map");
            const double weight = (intent == Intent::CONTINUE_STRAIGHT) ? kStraightBoost : 1.0;
            hyp.probability = kLaneletTotalConf * (weight / total_weight);
            hypotheses.push_back(std::move(hyp));
          }

          auto cv_poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);
          auto cv_hypothesis = buildHypothesis(std::move(cv_poses), time_step_, Intent::UNKNOWN);
          cv_hypothesis.probability = kCvFallbackConf;
          hypotheses.push_back(std::move(cv_hypothesis));

          RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "Cyclist prediction: %zu lanelet-based + 1 CV fallback, velocity=%.2f m/s",
            all_paths.size(),
            v);

          return hypotheses;
        }
      }
    }
  }

  return buildCvFallback(state, v, "no lanelet");
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
  const lanelet_msgs::msg::Lanelet & start_lanelet, int max_depth, const LaneletContext & ctx) const
{
  std::vector<std::pair<std::vector<Eigen::Vector2d>, Intent>> all_paths;

  if (start_lanelet.centerline.empty()) {
    return all_paths;
  }

  // Pre-cache start lanelet points for reuse across all paths
  std::vector<Eigen::Vector2d> start_path_points;
  start_path_points.reserve(start_lanelet.centerline.size());
  for (const auto & pt : start_lanelet.centerline) {
    start_path_points.emplace_back(pt.x, pt.y);
  }

  if (start_lanelet.successor_ids.empty()) {
    all_paths.emplace_back(std::move(start_path_points), Intent::CONTINUE_STRAIGHT);
    return all_paths;
  }

  const double kTurnAngleThreshold = cyclist_params_.turn_angle_threshold;
  const auto & sc = start_lanelet.centerline;

  for (int64_t successor_id : start_lanelet.successor_ids) {
    const auto * next_ll = findLaneletById(successor_id, ctx);
    if (!next_ll) {
      continue;
    }

    // Determine intent from heading change between start and next lanelet
    Intent intent = Intent::CONTINUE_STRAIGHT;
    const auto & nc = next_ll->centerline;
    if (sc.size() >= 2 && nc.size() >= 2) {
      const double start_heading =
        std::atan2(sc[sc.size() - 1].y - sc[sc.size() - 2].y, sc[sc.size() - 1].x - sc[sc.size() - 2].x);
      const double next_heading = std::atan2(nc[1].y - nc[0].y, nc[1].x - nc[0].x);
      const double angle_diff =
        std::atan2(std::sin(next_heading - start_heading), std::cos(next_heading - start_heading));
      if (angle_diff > kTurnAngleThreshold) {
        intent = Intent::TURN_LEFT;
      } else if (angle_diff < -kTurnAngleThreshold) {
        intent = Intent::TURN_RIGHT;
      }
    }

    // DFS: stack of (lanelet ptr, path-so-far, depth)
    using StackEntry = std::tuple<const lanelet_msgs::msg::Lanelet *, std::vector<Eigen::Vector2d>, int>;
    std::vector<StackEntry> stack;
    stack.emplace_back(next_ll, start_path_points, 0);

    while (!stack.empty()) {
      auto [current, current_path, depth] = std::move(stack.back());
      stack.pop_back();

      for (const auto & pt : current->centerline) {
        current_path.emplace_back(pt.x, pt.y);
      }

      if (depth + 1 >= max_depth) {
        if (current_path.size() >= 2) {
          all_paths.emplace_back(std::move(current_path), intent);
        }
        continue;
      }

      bool pushed_any = false;
      for (int64_t next_id : current->successor_ids) {
        const auto * next = findLaneletById(next_id, ctx);
        if (next) {
          stack.emplace_back(next, current_path, depth + 1);
          pushed_any = true;
        }
      }
      if (!pushed_any && current_path.size() >= 2) {
        all_paths.emplace_back(std::move(current_path), intent);
      }
    }
  }

  return all_paths;
}

}  // namespace prediction
