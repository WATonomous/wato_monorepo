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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace prediction {

namespace {
double extractYaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalizeAngle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}
} // namespace

TrajectoryPredictor::TrajectoryPredictor(rclcpp_lifecycle::LifecycleNode *node,
                                         double prediction_horizon,
                                         double time_step)
    : node_(node), prediction_horizon_(prediction_horizon),
      time_step_(time_step) {
  bicycle_model_ = std::make_unique<BicycleModel>();
  constant_velocity_model_ = std::make_unique<ConstantVelocityModel>();

  RCLCPP_INFO(node_->get_logger(), "TrajectoryPredictor initialized");
}

void TrajectoryPredictor::setLaneletAhead(
    const lanelet_msgs::msg::LaneletAhead::SharedPtr &msg) {
  lanelet_cache_ = *msg;
  lanelet_id_to_index_.clear();
  for (size_t i = 0; i < lanelet_cache_.lanelets.size(); ++i) {
    lanelet_id_to_index_[lanelet_cache_.lanelets[i].id] = i;
  }
}

void TrajectoryPredictor::setTemporaryLaneletData(
    const lanelet_msgs::msg::LaneletAhead &data) {
  lanelet_cache_backup_ = lanelet_cache_;
  lanelet_id_to_index_backup_ = lanelet_id_to_index_;

  lanelet_cache_ = data;
  lanelet_id_to_index_.clear();
  for (size_t i = 0; i < lanelet_cache_.lanelets.size(); ++i) {
    lanelet_id_to_index_[lanelet_cache_.lanelets[i].id] = i;
  }
}

void TrajectoryPredictor::restoreLaneletData() {
  lanelet_cache_ = lanelet_cache_backup_;
  lanelet_id_to_index_ = lanelet_id_to_index_backup_;
}

void TrajectoryPredictor::setLaneletQueryFunction(LaneletQueryFn fn) {
  lanelet_query_fn_ = std::move(fn);
}

lanelet_msgs::msg::LaneletAhead TrajectoryPredictor::queryVehicleLanelets(
    const std::string &vehicle_id, const geometry_msgs::msg::Point &position,
    double heading_rad) {
  // Check per-vehicle cache first
  auto it = vehicle_lanelet_cache_.find(vehicle_id);
  if (it != vehicle_lanelet_cache_.end()) {
    double dx = position.x - it->second.last_x;
    double dy = position.y - it->second.last_y;
    if (dx * dx + dy * dy <
        kVehicleCacheInvalidationDist * kVehicleCacheInvalidationDist) {
      return it->second.lanelet_ahead;
    }
  }

  // Cache miss or stale — fire async request (non-blocking).
  // Results arrive via updateVehicleLaneletCache() and will be
  // available on the next prediction cycle.
  if (lanelet_query_fn_) {
    lanelet_query_fn_(vehicle_id, position, heading_rad);
  }
  return {};
}

void TrajectoryPredictor::updateVehicleLaneletCache(
    const std::string &vehicle_id, const lanelet_msgs::msg::LaneletAhead &data,
    double x, double y) {
  vehicle_lanelet_cache_[vehicle_id] = {data, x, y};
}

double TrajectoryPredictor::estimateSpeed(
    const vision_msgs::msg::Detection3D &detection) {
  double length = detection.bbox.size.x;
  double default_speed = (length > 3.5) ? 5.0 : 1.4;

  rclcpp::Time now = node_->get_clock()->now();
  double x = detection.bbox.center.position.x;
  double y = detection.bbox.center.position.y;

  auto it = position_history_.find(detection.id);
  double speed = default_speed;

  if (it != position_history_.end()) {
    double dt = (now - it->second.stamp).seconds();
    if (dt > 0.01 && dt < 2.0) {
      double dx = x - it->second.x;
      double dy = y - it->second.y;
      double raw_speed = std::sqrt(dx * dx + dy * dy) / dt;
      // Clamp to a sane range: [0.5, 25.0] m/s for vehicles, [0.1, 3.0] for
      // pedestrians/cyclists
      if (length > 3.5) {
        speed = std::clamp(raw_speed, 0.5, 25.0);
      } else {
        speed = std::clamp(raw_speed, 0.1, 3.0);
      }
    }
  }

  // Update history
  position_history_[detection.id] = {x, y, now};

  return speed;
}

std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateHypotheses(
    const vision_msgs::msg::Detection3D &detection) {
  ObjectType obj_type = classifyObjectType(detection);

  // Estimate speed from position history — used by all vehicle hypothesis
  // generators so that both lanelet and geometric predictions are scaled
  // consistently by actual vehicle speed.
  double estimated_speed = estimateSpeed(detection);

  std::vector<TrajectoryHypothesis> hypotheses;

  switch (obj_type) {
  case ObjectType::VEHICLE:
    if (hasLaneletData()) {
      hypotheses = generateLaneletVehicleHypotheses(detection, estimated_speed);
    }
    // If ego cache didn't cover this vehicle, query per-vehicle lanelets
    if (hypotheses.empty() && lanelet_query_fn_) {
      double heading = extractYaw(detection.bbox.center.orientation);
      auto result = queryVehicleLanelets(
          detection.id, detection.bbox.center.position, heading);
      if (!result.lanelets.empty()) {
        setTemporaryLaneletData(result);
        hypotheses =
            generateLaneletVehicleHypotheses(detection, estimated_speed);
        restoreLaneletData();
      }
    }
    if (hypotheses.empty()) {
      hypotheses =
          generateGeometricVehicleHypotheses(detection, estimated_speed);
    }
    break;
  case ObjectType::PEDESTRIAN:
    hypotheses = generatePedestrianHypotheses(detection);
    break;
  case ObjectType::CYCLIST:
    hypotheses = generateCyclistHypotheses(detection);
    break;
  default:
    RCLCPP_WARN(node_->get_logger(),
                "Unknown object type, skipping prediction");
    break;
  }

  return hypotheses;
}

ObjectType TrajectoryPredictor::classifyObjectType(
    const vision_msgs::msg::Detection3D &detection) {
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

// =============================================================================
// Lanelet-aware vehicle hypothesis generation
// =============================================================================

std::vector<TrajectoryHypothesis>
TrajectoryPredictor::generateLaneletVehicleHypotheses(
    const vision_msgs::msg::Detection3D &detection, double speed) {
  std::vector<TrajectoryHypothesis> hypotheses;

  rclcpp::Time current_time = node_->get_clock()->now();
  std::string frame_id = "map";

  KinematicState initial_state;
  initial_state.x = detection.bbox.center.position.x;
  initial_state.y = detection.bbox.center.position.y;
  initial_state.theta = extractYaw(detection.bbox.center.orientation);
  initial_state.v = speed;
  initial_state.a = 0.0;
  initial_state.delta = 0.0;

  // Find which lanelet this vehicle is on
  LaneletMatch match = findVehicleLanelet(initial_state);
  if (!match.lanelet) {
    return hypotheses; // No match, caller falls back to geometric
  }

  const auto &current_ll = *match.lanelet;
  double required_distance = initial_state.v * prediction_horizon_ + 10.0;

  // =========================================================================
  // Curvature-aware scoring context
  // =========================================================================
  // On curved roads, heading deviation and lateral offset from centerline are
  // expected. Compute the road curvature so we can relax penalties accordingly.
  double road_curvature = computeCurvatureAtIndex(current_ll.centerline,
                                                  match.closest_centerline_idx);

  // Expected heading deviation on a curve: curvature * lookahead ≈ a few deg
  // Expected lateral offset on a curve: v^2 * curvature / (2 * lateral_accel)
  // We use these to discount heading_diff and lateral_offset in scoring.
  double curvature_heading_tolerance =
      std::min(road_curvature * 5.0, 0.3); // up to ~17 deg
  double curvature_lateral_tolerance =
      std::min(speed * speed * road_curvature / 8.0, 1.0); // up to 1m

  double adjusted_heading_diff =
      std::max(0.0, match.heading_diff - curvature_heading_tolerance);
  double adjusted_lateral_offset =
      (std::abs(match.lateral_offset) > curvature_lateral_tolerance)
          ? match.lateral_offset
          : match.lateral_offset * 0.3; // discount expected curve offset

  // =========================================================================
  // Heuristic 1: Stop persistence — stopped vehicles stay stopped
  // =========================================================================
  if (speed < 0.5) {
    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::STOP;
    hyp.probability = 3.0; // high raw score, will be normalized

    // Generate stationary trajectory
    for (double t = time_step_; t <= prediction_horizon_; t += time_step_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = frame_id;
      pose_stamped.header.stamp =
          current_time + rclcpp::Duration::from_seconds(t);
      pose_stamped.pose.position.x = initial_state.x;
      pose_stamped.pose.position.y = initial_state.y;
      pose_stamped.pose.position.z = 0.0;
      pose_stamped.pose.orientation = detection.bbox.center.orientation;
      hyp.poses.push_back(pose_stamped);
    }
    hypotheses.push_back(hyp);
  }

  // =========================================================================
  // Heuristic 10: Maneuver inertia — retrieve previous dominant intent
  // =========================================================================
  Intent prev_intent = Intent::UNKNOWN;
  auto prev_it = previous_intent_.find(detection.id);
  if (prev_it != previous_intent_.end()) {
    prev_intent = prev_it->second;
  }

  // =========================================================================
  // Heuristic 6/7: Lateral velocity for lane-change evidence
  // =========================================================================
  double lateral_vel =
      estimateLateralVelocity(detection.id, match.lateral_offset);

  // --- Hypothesis: Follow current lane (continue straight / follow road) ---
  {
    auto path = extractForwardPath(current_ll, match.closest_centerline_idx,
                                   required_distance);
    if (path.size() >= 2) {
      TrajectoryHypothesis hyp;
      hyp.header.stamp = current_time;
      hyp.header.frame_id = frame_id;

      // Determine intent based on lanelet geometry
      if (current_ll.is_intersection) {
        double entry_heading =
            std::atan2(path[1].y() - path[0].y(), path[1].x() - path[0].x());
        size_t last = path.size() - 1;
        double exit_heading = std::atan2(path[last].y() - path[last - 1].y(),
                                         path[last].x() - path[last - 1].x());
        double heading_change = normalizeAngle(exit_heading - entry_heading);
        if (heading_change > 0.3) {
          hyp.intent = Intent::TURN_LEFT;
        } else if (heading_change < -0.3) {
          hyp.intent = Intent::TURN_RIGHT;
        } else {
          hyp.intent = Intent::CONTINUE_STRAIGHT;
        }
      } else {
        hyp.intent = Intent::CONTINUE_STRAIGHT;
      }

      // Heuristic 1: Motion persistence — continuing trajectory gets high base
      // Heuristic 4: Lane-center preference — use curvature-adjusted offsets
      double lane_follow_prior = 2.5;
      hyp.probability = computeGeometricScore(
          adjusted_heading_diff, adjusted_lateral_offset, lane_follow_prior);

      hyp.poses = bicycle_model_->generateTrajectory(
          initial_state, path, prediction_horizon_, time_step_, current_time,
          frame_id);

      // Heuristic 9: Smoothness preference — penalize rough trajectories
      if (!hyp.poses.empty()) {
        double smoothness = computeTrajectorySmoothness(hyp.poses);
        // Smooth trajectories (low curvature) get higher score
        double smoothness_factor = std::exp(-smoothness * 2.0);
        hyp.probability *= (0.5 + 0.5 * smoothness_factor);

        // Heuristic 10: Maneuver inertia — boost if continuing previous intent
        if (prev_intent != Intent::UNKNOWN && prev_intent == hyp.intent) {
          hyp.probability *= 1.3;
        }

        hypotheses.push_back(hyp);
      }
    }
  }

  // --- Hypothesis: Follow each successor lanelet (for intersections with
  // multiple exits) ---
  if (current_ll.successor_ids.size() > 1) {
    for (int64_t succ_id : current_ll.successor_ids) {
      const auto *succ_ll = findLaneletById(succ_id);
      if (!succ_ll || succ_ll->centerline.size() < 2)
        continue;

      // Build path: remaining current centerline + successor centerline
      std::vector<Eigen::Vector2d> path;
      for (size_t i = match.closest_centerline_idx;
           i < current_ll.centerline.size(); ++i) {
        path.emplace_back(current_ll.centerline[i].x,
                          current_ll.centerline[i].y);
      }
      for (const auto &pt : succ_ll->centerline) {
        path.emplace_back(pt.x, pt.y);
      }
      if (computePathLength(path) < required_distance) {
        for (int64_t succ_succ_id : succ_ll->successor_ids) {
          const auto *ss = findLaneletById(succ_succ_id);
          if (ss) {
            for (const auto &pt : ss->centerline) {
              path.emplace_back(pt.x, pt.y);
            }
            if (computePathLength(path) >= required_distance)
              break;
          }
        }
      }

      if (path.size() < 2)
        continue;

      TrajectoryHypothesis hyp;
      hyp.header.stamp = current_time;
      hyp.header.frame_id = frame_id;

      double entry_heading =
          std::atan2(path[1].y() - path[0].y(), path[1].x() - path[0].x());
      size_t last = path.size() - 1;
      double exit_heading = std::atan2(path[last].y() - path[last - 1].y(),
                                       path[last].x() - path[last - 1].x());
      double heading_change = normalizeAngle(exit_heading - entry_heading);

      if (heading_change > 0.3) {
        hyp.intent = Intent::TURN_LEFT;
      } else if (heading_change < -0.3) {
        hyp.intent = Intent::TURN_RIGHT;
      } else {
        hyp.intent = Intent::CONTINUE_STRAIGHT;
      }

      // Heuristic 8: Route commitment — lower prior for turns
      double maneuver_prior = (std::abs(heading_change) > 0.3) ? 0.5 : 1.5;
      hyp.probability = computeGeometricScore(
          adjusted_heading_diff, adjusted_lateral_offset, maneuver_prior);

      hyp.poses = bicycle_model_->generateTrajectory(
          initial_state, path, prediction_horizon_, time_step_, current_time,
          frame_id);

      if (!hyp.poses.empty()) {
        // Heuristic 9: Smoothness preference
        double smoothness = computeTrajectorySmoothness(hyp.poses);
        double smoothness_factor = std::exp(-smoothness * 2.0);
        hyp.probability *= (0.5 + 0.5 * smoothness_factor);

        // Heuristic 10: Maneuver inertia
        if (prev_intent != Intent::UNKNOWN && prev_intent == hyp.intent) {
          hyp.probability *= 1.3;
        }

        hypotheses.push_back(hyp);
      }
    }
  }

  // --- Hypothesis: Lane change left ---
  if (current_ll.can_change_left && current_ll.left_lane_id > 0) {
    auto path =
        extractLaneChangePath(current_ll, match.closest_centerline_idx,
                              current_ll.left_lane_id, required_distance);
    if (path.size() >= 2) {
      TrajectoryHypothesis hyp;
      hyp.header.stamp = current_time;
      hyp.header.frame_id = frame_id;
      hyp.intent = Intent::LANE_CHANGE_LEFT;

      // Heuristic 3: Lane-change completion — require sustained lateral
      // velocity toward the target lane, not just positional offset which can
      // occur naturally on curves.
      double lc_prior = 0.15; // base: lane changes are uncommon
      // Only boost if lateral velocity is consistently toward the left lane
      if (lateral_vel > 0.3) {
        lc_prior = 0.6; // active lateral motion toward left
      }

      // Heuristic 8: Route commitment — suppress lane changes in
      // intersections
      if (current_ll.is_intersection) {
        lc_prior *= 0.1;
      }

      // Heuristic 7: Curvature suppression — on curves, lateral offset is
      // expected and should NOT trigger lane-change. Scale down the prior
      // proportionally to road curvature.
      if (road_curvature > 0.005) { // ~200m radius or tighter
        lc_prior *= std::max(0.1, 1.0 - road_curvature * 50.0);
      }

      hyp.probability = computeGeometricScore(
          adjusted_heading_diff, adjusted_lateral_offset, lc_prior);

      hyp.poses = bicycle_model_->generateTrajectory(
          initial_state, path, prediction_horizon_, time_step_, current_time,
          frame_id);

      if (!hyp.poses.empty()) {
        // Heuristic 9: Smoothness preference
        double smoothness = computeTrajectorySmoothness(hyp.poses);
        double smoothness_factor = std::exp(-smoothness * 2.0);
        hyp.probability *= (0.5 + 0.5 * smoothness_factor);

        // Heuristic 10: Maneuver inertia — boost if already changing left,
        // penalize contradictory switch from right
        if (prev_intent == Intent::LANE_CHANGE_LEFT) {
          hyp.probability *= 1.5; // Heuristic 3: completing a lane change
        } else if (prev_intent == Intent::LANE_CHANGE_RIGHT) {
          hyp.probability *= 0.1; // Heuristic 10: rarely switch directions
        }

        hypotheses.push_back(hyp);
      }
    }
  }

  // --- Hypothesis: Lane change right ---
  if (current_ll.can_change_right && current_ll.right_lane_id > 0) {
    auto path =
        extractLaneChangePath(current_ll, match.closest_centerline_idx,
                              current_ll.right_lane_id, required_distance);
    if (path.size() >= 2) {
      TrajectoryHypothesis hyp;
      hyp.header.stamp = current_time;
      hyp.header.frame_id = frame_id;
      hyp.intent = Intent::LANE_CHANGE_RIGHT;

      double lc_prior = 0.15;
      if (lateral_vel < -0.3) {
        lc_prior = 0.6; // active lateral motion toward right
      }

      if (current_ll.is_intersection) {
        lc_prior *= 0.1;
      }

      if (road_curvature > 0.005) {
        lc_prior *= std::max(0.1, 1.0 - road_curvature * 50.0);
      }

      hyp.probability = computeGeometricScore(
          adjusted_heading_diff, adjusted_lateral_offset, lc_prior);

      hyp.poses = bicycle_model_->generateTrajectory(
          initial_state, path, prediction_horizon_, time_step_, current_time,
          frame_id);

      if (!hyp.poses.empty()) {
        double smoothness = computeTrajectorySmoothness(hyp.poses);
        double smoothness_factor = std::exp(-smoothness * 2.0);
        hyp.probability *= (0.5 + 0.5 * smoothness_factor);

        if (prev_intent == Intent::LANE_CHANGE_RIGHT) {
          hyp.probability *= 1.5;
        } else if (prev_intent == Intent::LANE_CHANGE_LEFT) {
          hyp.probability *= 0.1;
        }

        hypotheses.push_back(hyp);
      }
    }
  }

  // Normalize probabilities
  if (!hypotheses.empty()) {
    double total = 0.0;
    for (const auto &h : hypotheses)
      total += h.probability;
    if (total > 1e-6) {
      for (auto &h : hypotheses)
        h.probability /= total;
    } else {
      double uniform = 1.0 / hypotheses.size();
      for (auto &h : hypotheses)
        h.probability = uniform;
    }

    // Record the dominant intent for maneuver inertia on next cycle
    double best_prob = 0.0;
    Intent best_intent = Intent::UNKNOWN;
    for (const auto &h : hypotheses) {
      if (h.probability > best_prob) {
        best_prob = h.probability;
        best_intent = h.intent;
      }
    }
    previous_intent_[detection.id] = best_intent;
  }

  RCLCPP_DEBUG(node_->get_logger(),
               "Generated %zu lanelet-based vehicle hypotheses (lanelet %ld)",
               hypotheses.size(), current_ll.id);

  return hypotheses;
}

// =============================================================================
// Lanelet helper implementations
// =============================================================================

TrajectoryPredictor::LaneletMatch
TrajectoryPredictor::findVehicleLanelet(const KinematicState &state) const {
  LaneletMatch best_match{nullptr, 0, std::numeric_limits<double>::max(), M_PI};
  double best_score = std::numeric_limits<double>::max();

  for (const auto &ll : lanelet_cache_.lanelets) {
    if (ll.centerline.size() < 2)
      continue;

    for (size_t i = 0; i < ll.centerline.size(); ++i) {
      double dx = ll.centerline[i].x - state.x;
      double dy = ll.centerline[i].y - state.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist > 10.0)
        continue; // Skip far-away points

      // Compute heading at this centerline point
      double cl_heading = computeHeadingAtIndex(ll.centerline, i);
      double heading_diff = std::abs(normalizeAngle(state.theta - cl_heading));

      // Combined score: distance + heading penalty
      double score = dist + 3.0 * heading_diff;

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
  if (best_match.lanelet && best_match.heading_diff > M_PI / 3.0) {
    best_match.lanelet = nullptr;
  }

  return best_match;
}

std::vector<Eigen::Vector2d> TrajectoryPredictor::extractForwardPath(
    const lanelet_msgs::msg::Lanelet &lanelet, size_t start_idx,
    double required_distance) const {
  std::vector<Eigen::Vector2d> path;

  // Add remaining centerline points from current lanelet
  for (size_t i = start_idx; i < lanelet.centerline.size(); ++i) {
    path.emplace_back(lanelet.centerline[i].x, lanelet.centerline[i].y);
  }

  // Extend through successors if needed
  double accumulated = computePathLength(path);
  std::vector<int64_t> to_visit = {lanelet.successor_ids.begin(),
                                   lanelet.successor_ids.end()};
  size_t visit_idx = 0;

  while (accumulated < required_distance && visit_idx < to_visit.size()) {
    const auto *succ = findLaneletById(to_visit[visit_idx]);
    ++visit_idx;
    if (!succ)
      continue;

    for (const auto &pt : succ->centerline) {
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
    const lanelet_msgs::msg::Lanelet &current_lanelet, size_t start_idx,
    int64_t target_lane_id, double required_distance) const {
  const auto *target_ll = findLaneletById(target_lane_id);
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
    path.emplace_back(current_lanelet.centerline[i].x,
                      current_lanelet.centerline[i].y);
  }

  // Find closest point on target lane to the transition point
  if (path.empty())
    return {};
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
  size_t blend_count = 5;
  if (target_start + blend_count < target_ll->centerline.size() &&
      !path.empty()) {
    Eigen::Vector2d from = path.back();
    for (size_t i = 0; i < blend_count; ++i) {
      double t = static_cast<double>(i + 1) / static_cast<double>(blend_count);
      size_t tgt_idx = target_start + i;
      if (tgt_idx >= target_ll->centerline.size())
        break;
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
    const auto *succ = findLaneletById(target_ll->successor_ids[0]);
    if (succ) {
      for (const auto &pt : succ->centerline) {
        path.emplace_back(pt.x, pt.y);
      }
    }
  }

  return path;
}

const lanelet_msgs::msg::Lanelet *
TrajectoryPredictor::findLaneletById(int64_t id) const {
  auto it = lanelet_id_to_index_.find(id);
  if (it != lanelet_id_to_index_.end() &&
      it->second < lanelet_cache_.lanelets.size()) {
    return &lanelet_cache_.lanelets[it->second];
  }
  return nullptr;
}

double TrajectoryPredictor::computePathLength(
    const std::vector<Eigen::Vector2d> &path) const {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}

double TrajectoryPredictor::computeHeadingAtIndex(
    const std::vector<geometry_msgs::msg::Point> &centerline,
    size_t idx) const {
  if (centerline.size() < 2)
    return 0.0;

  size_t i0, i1;
  if (idx + 1 < centerline.size()) {
    i0 = idx;
    i1 = idx + 1;
  } else {
    i0 = idx - 1;
    i1 = idx;
  }

  return std::atan2(centerline[i1].y - centerline[i0].y,
                    centerline[i1].x - centerline[i0].x);
}

double TrajectoryPredictor::computeGeometricScore(double heading_diff,
                                                  double lateral_offset,
                                                  double maneuver_prior) const {
  // Gaussian scoring: higher when heading aligns and vehicle is near centerline
  double heading_score = std::exp(-heading_diff * heading_diff / 0.2);
  double lateral_score = std::exp(-lateral_offset * lateral_offset / 4.0);
  return heading_score * lateral_score * maneuver_prior;
}

double TrajectoryPredictor::computeCurvatureAtIndex(
    const std::vector<geometry_msgs::msg::Point> &centerline,
    size_t idx) const {
  if (centerline.size() < 3) {
    return 0.0;
  }
  // Use three-point discrete curvature: kappa = 2*|cross(p1-p0, p2-p1)| /
  // (|p1-p0| * |p2-p1| * |p2-p0|)
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
  double lc = std::sqrt((centerline[i2].x - centerline[i0].x) *
                            (centerline[i2].x - centerline[i0].x) +
                        (centerline[i2].y - centerline[i0].y) *
                            (centerline[i2].y - centerline[i0].y));

  double denom = la * lb * lc;
  if (denom < 1e-9) {
    return 0.0;
  }
  return 2.0 * cross / denom;
}

double TrajectoryPredictor::computeTrajectorySmoothness(
    const std::vector<geometry_msgs::msg::PoseStamped> &poses) const {
  if (poses.size() < 3) {
    return 0.0;
  }
  // Mean absolute heading change rate (approximate curvature)
  double total_curvature = 0.0;
  for (size_t i = 1; i + 1 < poses.size(); ++i) {
    double dx1 = poses[i].pose.position.x - poses[i - 1].pose.position.x;
    double dy1 = poses[i].pose.position.y - poses[i - 1].pose.position.y;
    double dx2 = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    double dy2 = poses[i + 1].pose.position.y - poses[i].pose.position.y;

    double h1 = std::atan2(dy1, dx1);
    double h2 = std::atan2(dy2, dx2);
    double dh = h2 - h1;
    while (dh > M_PI)
      dh -= 2.0 * M_PI;
    while (dh < -M_PI)
      dh += 2.0 * M_PI;

    total_curvature += std::abs(dh);
  }
  return total_curvature / static_cast<double>(poses.size() - 2);
}

double
TrajectoryPredictor::estimateLateralVelocity(const std::string &vehicle_id,
                                             double lateral_offset) {
  rclcpp::Time now = node_->get_clock()->now();
  double lateral_vel = 0.0;

  auto it = lateral_offset_history_.find(vehicle_id);
  if (it != lateral_offset_history_.end()) {
    double dt = (now - it->second.stamp).seconds();
    if (dt > 0.01 && dt < 2.0) {
      lateral_vel = (lateral_offset - it->second.lateral_offset) / dt;
    }
  }

  lateral_offset_history_[vehicle_id] = {lateral_offset, now};
  return lateral_vel;
}

// =============================================================================
// Geometric fallback (original behavior, used when no lanelet data)
// =============================================================================

std::vector<TrajectoryHypothesis>
TrajectoryPredictor::generateGeometricVehicleHypotheses(
    const vision_msgs::msg::Detection3D &detection, double speed) {
  std::vector<TrajectoryHypothesis> hypotheses;

  rclcpp::Time current_time = node_->get_clock()->now();
  std::string frame_id = "map";

  KinematicState initial_state;
  initial_state.x = detection.bbox.center.position.x;
  initial_state.y = detection.bbox.center.position.y;
  initial_state.theta = extractYaw(detection.bbox.center.orientation);
  initial_state.v = speed;
  initial_state.a = 0.0;
  initial_state.delta = 0.0;

  // Continue Straight
  {
    std::vector<Eigen::Vector2d> straight_path;
    for (double d = 0.0; d <= initial_state.v * prediction_horizon_ + 10.0;
         d += 1.0) {
      straight_path.emplace_back(
          initial_state.x + d * std::cos(initial_state.theta),
          initial_state.y + d * std::sin(initial_state.theta));
    }

    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::CONTINUE_STRAIGHT;
    hyp.probability = 0.6;
    hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, straight_path, prediction_horizon_, time_step_,
        current_time, frame_id);
    hypotheses.push_back(hyp);
  }

  // Turn Left
  {
    std::vector<Eigen::Vector2d> left_path;
    double turn_radius = 10.0;
    for (double angle = 0.0; angle <= M_PI / 2.0; angle += 0.1) {
      double path_x =
          initial_state.x +
          turn_radius * std::sin(angle) * std::cos(initial_state.theta) -
          turn_radius * (1.0 - std::cos(angle)) * std::sin(initial_state.theta);
      double path_y =
          initial_state.y +
          turn_radius * std::sin(angle) * std::sin(initial_state.theta) +
          turn_radius * (1.0 - std::cos(angle)) * std::cos(initial_state.theta);
      left_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::TURN_LEFT;
    hyp.probability = 0.1;
    hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, left_path, prediction_horizon_, time_step_, current_time,
        frame_id);
    hypotheses.push_back(hyp);
  }

  // Turn Right
  {
    std::vector<Eigen::Vector2d> right_path;
    double turn_radius = 10.0;
    for (double angle = 0.0; angle <= M_PI / 2.0; angle += 0.1) {
      double path_x =
          initial_state.x +
          turn_radius * std::sin(angle) * std::cos(initial_state.theta) +
          turn_radius * (1.0 - std::cos(angle)) * std::sin(initial_state.theta);
      double path_y =
          initial_state.y +
          turn_radius * std::sin(angle) * std::sin(initial_state.theta) -
          turn_radius * (1.0 - std::cos(angle)) * std::cos(initial_state.theta);
      right_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::TURN_RIGHT;
    hyp.probability = 0.1;
    hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, right_path, prediction_horizon_, time_step_,
        current_time, frame_id);
    hypotheses.push_back(hyp);
  }

  // Lane Change Left
  {
    std::vector<Eigen::Vector2d> lcl_path;
    double lane_width = 3.5;
    double lc_distance = 30.0;
    for (double d = 0.0; d <= lc_distance; d += 1.0) {
      double lateral_offset = lane_width * (d / lc_distance);
      double path_x = initial_state.x + d * std::cos(initial_state.theta) -
                      lateral_offset * std::sin(initial_state.theta);
      double path_y = initial_state.y + d * std::sin(initial_state.theta) +
                      lateral_offset * std::cos(initial_state.theta);
      lcl_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::LANE_CHANGE_LEFT;
    hyp.probability = 0.1;
    hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, lcl_path, prediction_horizon_, time_step_, current_time,
        frame_id);
    hypotheses.push_back(hyp);
  }

  // Lane Change Right
  {
    std::vector<Eigen::Vector2d> lcr_path;
    double lane_width = 3.5;
    double lc_distance = 30.0;
    for (double d = 0.0; d <= lc_distance; d += 1.0) {
      double lateral_offset = lane_width * (d / lc_distance);
      double path_x = initial_state.x + d * std::cos(initial_state.theta) +
                      lateral_offset * std::sin(initial_state.theta);
      double path_y = initial_state.y + d * std::sin(initial_state.theta) -
                      lateral_offset * std::cos(initial_state.theta);
      lcr_path.emplace_back(path_x, path_y);
    }

    TrajectoryHypothesis hyp;
    hyp.header.stamp = current_time;
    hyp.header.frame_id = frame_id;
    hyp.intent = Intent::LANE_CHANGE_RIGHT;
    hyp.probability = 0.1;
    hyp.poses = bicycle_model_->generateTrajectory(
        initial_state, lcr_path, prediction_horizon_, time_step_, current_time,
        frame_id);
    hypotheses.push_back(hyp);
  }

  RCLCPP_DEBUG(node_->get_logger(),
               "Generated %zu geometric vehicle hypotheses (no lanelet data)",
               hypotheses.size());

  return hypotheses;
}

// =============================================================================
// Pedestrian and Cyclist hypotheses (unchanged)
// =============================================================================

std::vector<TrajectoryHypothesis>
TrajectoryPredictor::generatePedestrianHypotheses(
    const vision_msgs::msg::Detection3D &detection) {
  std::vector<TrajectoryHypothesis> hypotheses;

  rclcpp::Time current_time = node_->get_clock()->now();
  std::string frame_id = "map";

  TrajectoryHypothesis walk_hyp;
  walk_hyp.header.stamp = current_time;
  walk_hyp.header.frame_id = frame_id;
  walk_hyp.intent = Intent::CONTINUE_STRAIGHT;
  walk_hyp.probability = 1.0;

  double current_x = detection.bbox.center.position.x;
  double current_y = detection.bbox.center.position.y;
  double current_z = detection.bbox.center.position.z;
  double heading = extractYaw(detection.bbox.center.orientation);

  double length = detection.bbox.size.x;
  double velocity = (length > 3.5) ? 5.0 : 1.4;

  for (double t = time_step_; t <= prediction_horizon_; t += time_step_) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp =
        current_time + rclcpp::Duration::from_seconds(t);
    pose_stamped.pose.position.x = current_x + velocity * std::cos(heading) * t;
    pose_stamped.pose.position.y = current_y + velocity * std::sin(heading) * t;
    pose_stamped.pose.position.z = current_z;
    pose_stamped.pose.orientation = detection.bbox.center.orientation;

    walk_hyp.poses.push_back(pose_stamped);
  }

  hypotheses.push_back(walk_hyp);
  return hypotheses;
}

std::vector<TrajectoryHypothesis>
TrajectoryPredictor::generateCyclistHypotheses(
    const vision_msgs::msg::Detection3D &detection) {
  std::vector<TrajectoryHypothesis> hypotheses;

  rclcpp::Time current_time = node_->get_clock()->now();
  std::string frame_id = "map";

  TrajectoryHypothesis cycle_hyp;
  cycle_hyp.header.stamp = current_time;
  cycle_hyp.header.frame_id = frame_id;
  cycle_hyp.intent = Intent::CONTINUE_STRAIGHT;
  cycle_hyp.probability = 1.0;

  double current_x = detection.bbox.center.position.x;
  double current_y = detection.bbox.center.position.y;
  double current_z = detection.bbox.center.position.z;
  double heading = extractYaw(detection.bbox.center.orientation);

  double length = detection.bbox.size.x;
  double velocity = (length > 3.5) ? 5.0 : 1.4;

  for (double t = time_step_; t <= prediction_horizon_; t += time_step_) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp =
        current_time + rclcpp::Duration::from_seconds(t);
    pose_stamped.pose.position.x = current_x + velocity * std::cos(heading) * t;
    pose_stamped.pose.position.y = current_y + velocity * std::sin(heading) * t;
    pose_stamped.pose.position.z = current_z;
    pose_stamped.pose.orientation = detection.bbox.center.orientation;

    cycle_hyp.poses.push_back(pose_stamped);
  }

  hypotheses.push_back(cycle_hyp);
  return hypotheses;
}

} // namespace prediction
