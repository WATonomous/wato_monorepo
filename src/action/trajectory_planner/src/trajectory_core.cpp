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

#include "trajectory_planner/trajectory_core.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

// For pure C++ logging without node dependency, we can use iostream or rclcpp::Logger if passed.
// For now, let's stick to safe defaults.

namespace trajectory_planner
{

static constexpr int8_t LETHAL_COST = 100;

// Extracts yaw from a pose's quaternion, assuming a planar (roll = pitch = 0) orientation.
// Shared by compute_trajectory (curvature-based speed limiting) and deform_path (computing
// each point's left-normal direction), so it's pulled out here instead of duplicated.
static double yaw_from_pose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Builds a planar (roll = pitch = 0) quaternion representing a rotation of `yaw` (radians)
// about the z-axis — the inverse of yaw_from_pose above. Used by deform_path to rebuild each
// point's orientation after its position has moved.
static geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

TrajectoryCore::TrajectoryCore(const TrajectoryConfig & config)
: config_(config)
{}

wato_trajectory_msgs::msg::Trajectory TrajectoryCore::compute_trajectory(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & costmap,
  double limit_speed,
  double current_speed)
{
  wato_trajectory_msgs::msg::Trajectory trajectory;
  trajectory.header = path.header;

  if (path.poses.empty()) {
    return trajectory;
  }

  // Nudge the path laterally around any slight obstacles first. Everything below — collision
  // distance, braking, curvature-based speed limiting, the output poses — then operates on
  // this deformed geometry instead of the raw planner output.
  const nav_msgs::msg::Path deformed_path = deform_path(path, costmap);

  std::optional<double> obstacle_dist = find_first_collision(deformed_path, costmap);

  double effective_max_speed = std::max(0.0, limit_speed);
  double dist_along_path = 0.0;
  double prev_speed = current_speed;
  auto prev_pos = deformed_path.poses.front().pose.position;

  for (size_t i = 0; i < deformed_path.poses.size(); ++i) {
    const auto & pos = deformed_path.poses[i].pose.position;

    double curr_dx = pos.x - prev_pos.x;
    double curr_dy = pos.y - prev_pos.y;
    double segment_dist = std::hypot(curr_dx, curr_dy);
    dist_along_path += segment_dist;
    prev_pos = pos;

    double target_speed = effective_max_speed;

    double tang_accel =
      segment_dist > 0.0 ? (target_speed * target_speed - prev_speed * prev_speed) / (2.0 * segment_dist) : 0.0;

    if (std::fabs(tang_accel) > config_.max_tangential_accel || segment_dist == 0.0) {
      double signed_accel = std::copysign(config_.max_tangential_accel, tang_accel);
      double v_sq = prev_speed * prev_speed + 2.0 * signed_accel * segment_dist;
      target_speed = std::sqrt(std::max(0.0, v_sq));
    }

    if (i > 0 && segment_dist > 1e-9) {
      double yaw_curr = yaw_from_pose(deformed_path.poses[i].pose);
      double yaw_prev = yaw_from_pose(deformed_path.poses[i - 1].pose);
      double delta_theta = std::fabs(std::remainder(yaw_curr - yaw_prev, 2.0 * M_PI));
      if (delta_theta > 1e-6) {
        double radius = segment_dist / delta_theta;
        if ((target_speed * target_speed) / radius > config_.max_lateral_accel) {
          target_speed = std::sqrt(config_.max_lateral_accel * radius);
        }
      }
    }

    if (obstacle_dist.has_value()) {
      double dist_remaining = *obstacle_dist - dist_along_path;
      if (dist_remaining <= config_.stop_distance) {
        target_speed = 0.0;
      } else {
        double braking_dist = dist_remaining - config_.stop_distance;
        double comfort_stop_dist = prev_speed * prev_speed / (2.0 * config_.max_tangential_accel);

        double effective_accel = config_.max_tangential_accel;
        if (comfort_stop_dist > 0.0 && braking_dist < comfort_stop_dist) {
          double blend = braking_dist / comfort_stop_dist;
          effective_accel =
            config_.max_tangential_accel + (1.0 - blend) * (config_.max_emergency_accel - config_.max_tangential_accel);
        }

        double v_brake = std::sqrt(2.0 * effective_accel * braking_dist);
        target_speed = std::min(target_speed, v_brake);
      }
    }

    // Scale speed down for non-lethal costmap costs
    double yaw = yaw_from_pose(deformed_path.poses[i].pose);
    int8_t cost = get_max_footprint_cost(pos.x, pos.y, yaw, costmap);
    if (cost > 0 && cost < LETHAL_COST) {
      double cost_scale = 1.0 - static_cast<double>(cost) / LETHAL_COST;
      target_speed *= cost_scale;
    }

    target_speed = std::clamp(target_speed, 0.0, effective_max_speed);
    prev_speed = target_speed;

    wato_trajectory_msgs::msg::TrajectoryPoint point;
    point.pose = deformed_path.poses[i].pose;
    point.max_speed = target_speed;
    trajectory.points.push_back(point);
  }

  // Backward pass: propagate speed limits backward so the vehicle decelerates
  // in advance of curves, obstacles, and stops instead of braking suddenly.
  if (trajectory.points.size() >= 2) {
    for (int i = static_cast<int>(trajectory.points.size()) - 2; i >= 0; --i) {
      double dx = deformed_path.poses[i + 1].pose.position.x - deformed_path.poses[i].pose.position.x;
      double dy = deformed_path.poses[i + 1].pose.position.y - deformed_path.poses[i].pose.position.y;
      double seg = std::hypot(dx, dy);
      if (seg > 1e-9) {
        double next_speed = trajectory.points[i + 1].max_speed;
        double v_max = std::sqrt(next_speed * next_speed + 2.0 * config_.max_tangential_accel * seg);
        trajectory.points[i].max_speed = std::min(trajectory.points[i].max_speed, v_max);
      }
    }
  }

  return trajectory;
}

std::optional<double> TrajectoryCore::find_first_collision(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap)
{
  if (path.poses.empty()) {
    return std::nullopt;
  }

  double total_dist = 0.0;
  auto prev_pose = path.poses.front().pose.position;

  // Map parameters
  double map_origin_x = costmap.info.origin.position.x;
  double map_origin_y = costmap.info.origin.position.y;
  double resolution = costmap.info.resolution;
  int width = costmap.info.width;
  int height = costmap.info.height;

  auto get_cost = [&](double x, double y) -> int8_t {
    int cx = (x - map_origin_x) / resolution;
    int cy = (y - map_origin_y) / resolution;
    if (cx < 0 || cx >= width || cy < 0 || cy >= height) return -1;
    return costmap.data[cy * width + cx];
  };

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & curr_pt = path.poses[i].pose.position;

    double segment_len = 0.0;
    if (i > 0) {
      segment_len = std::hypot(curr_pt.x - prev_pose.x, curr_pt.y - prev_pose.y);
    }

    // Interpolate
    int steps = std::max(1, static_cast<int>(std::ceil(segment_len / config_.interpolation_resolution)));

    for (int j = 0; j < steps; ++j) {
      double s = static_cast<double>(j) / steps;
      double interp_x = prev_pose.x + s * (curr_pt.x - prev_pose.x);
      double interp_y = prev_pose.y + s * (curr_pt.y - prev_pose.y);

      // Check the vehicle bounding box corners rotated to path heading for lethal obstacles.
      double theta = std::atan2(curr_pt.y - prev_pose.y, curr_pt.x - prev_pose.x);
      double cos_t = std::cos(theta);
      double sin_t = std::sin(theta);

      auto check_corner = [&](double lx, double ly) -> bool {
        double wx = interp_x + lx * cos_t - ly * sin_t;
        double wy = interp_y + lx * sin_t + ly * cos_t;
        return get_cost(wx, wy) >= LETHAL_COST;
      };

      // Sweep the full front edge of the vehicle footprint
      for (double ly = config_.footprint_y_min; ly <= config_.footprint_y_max + 1e-9; ly += resolution) {
        double clamped_ly = std::min(ly, config_.footprint_y_max);
        if (check_corner(config_.footprint_x_max, clamped_ly)) return total_dist + (s * segment_len);
      }
      // Keep rear corners for side / rear obstacle coverage.
      if (check_corner(config_.footprint_x_min, config_.footprint_y_min)) return total_dist + (s * segment_len);
      if (check_corner(config_.footprint_x_min, config_.footprint_y_max)) return total_dist + (s * segment_len);
    }

    total_dist += segment_len;
    prev_pose = curr_pt;
  }

  return std::nullopt;
}

nav_msgs::msg::Path TrajectoryCore::deform_path(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap)
{
  // Endpoints are always pinned in place, so we need at least one interior point to move.
  if (path.poses.size() < 3) {
    return path;
  }

  const size_t n = path.poses.size();

  // Per-point lateral offset along the left-normal, in metres. Positive = left, negative =
  // right. Starts at zero everywhere; the endpoints (index 0 and n-1) are never touched below,
  // so they stay pinned to zero throughout.
  std::vector<double> offset(n, 0.0);

  // Left-normal direction at each point, derived from the path's own heading. Rotating the
  // tangent (cos(yaw), sin(yaw)) by +90 degrees gives (-sin(yaw), cos(yaw)).
  std::vector<std::pair<double, double>> normal(n);
  for (size_t i = 0; i < n; ++i) {
    double yaw = yaw_from_pose(path.poses[i].pose);
    normal[i] = {-std::sin(yaw), std::cos(yaw)};
  }

  // For each interior point, find the offset it should end up at: 0 if the point is already
  // clear, or the cheapest non-lethal spot found by checking candidates across the full
  // allowed range if it isn't. This runs once, up front — not repeated every smoothing
  // iteration below — since it only depends on the original path and the costmap, neither of
  // which change during smoothing.
  std::vector<double> desired(n, 0.0);
  for (size_t i = 1; i + 1 < n; ++i) {
    const auto & pos = path.poses[i].pose.position;
    double yaw = yaw_from_pose(path.poses[i].pose);

    int8_t cost_here = get_max_footprint_cost(pos.x, pos.y, yaw, costmap);
    if (cost_here <= config_.deformation_trigger_cost) {
      continue;  // already clear enough — desired[i] stays 0
    }

    double best_offset = 0.0;
    int8_t best_cost = cost_here;
    for (double candidate = -config_.max_lateral_shift; candidate <= config_.max_lateral_shift + 1e-9;
         candidate += config_.lateral_search_step)
    {
      double cx = pos.x + candidate * normal[i].first;
      double cy = pos.y + candidate * normal[i].second;
      int8_t cost = get_max_footprint_cost(cx, cy, yaw, costmap);
      if (cost >= LETHAL_COST) {
        continue;  // never move somewhere that would collide
      }
      bool cheaper = cost < best_cost;
      bool tied_but_closer_to_centre = cost == best_cost && std::fabs(candidate) < std::fabs(best_offset);
      if (cheaper || tied_but_closer_to_centre) {
        best_cost = cost;
        best_offset = candidate;
      }
    }
    desired[i] = best_offset;
  }

  // Relax offset[] toward desired[] over several passes. Each point is nudged by two forces:
  // a smoothing term that pulls it toward the average of its two neighbours (prevents kinks),
  // and a pull term that pulls it toward its own target offset found above. A per-point safety
  // check then rejects any single step that would land the point somewhere lethal, reverting
  // it to its previous (already-checked-safe) offset instead.
  for (int iter = 0; iter < config_.deformation_iterations; ++iter) {
    double max_disp = 0.0;
    for (size_t i = 1; i + 1 < n; ++i) {
      double smoothing_force = config_.smoothing_gain * (offset[i - 1] + offset[i + 1] - 2.0 * offset[i]);
      double pull_force = config_.pull_gain * (desired[i] - offset[i]);
      double step = std::clamp(smoothing_force + pull_force, -config_.max_step, config_.max_step);
      double new_offset = std::clamp(offset[i] + step, -config_.max_lateral_shift, config_.max_lateral_shift);

      const auto & pos = path.poses[i].pose.position;
      double yaw = yaw_from_pose(path.poses[i].pose);
      double nx = pos.x + new_offset * normal[i].first;
      double ny = pos.y + new_offset * normal[i].second;
      if (get_max_footprint_cost(nx, ny, yaw, costmap) >= LETHAL_COST) {
        new_offset = offset[i];  // this step would collide — stay at the previous, safe offset
      }

      max_disp = std::max(max_disp, std::fabs(new_offset - offset[i]));
      offset[i] = new_offset;
    }
    if (max_disp < config_.deformation_convergence_tol) {
      break;  // nothing moved meaningfully this pass — converged early
    }
  }

  // Skip rebuilding the path if every offset is still at its initial value of zero — the
  // common case, since offset[i] only moves for points that were triggered above.
  double max_offset = 0.0;
  for (double d : offset) {
    max_offset = std::max(max_offset, std::fabs(d));
  }
  if (max_offset < 1e-6) {
    return path;
  }

  // Shift every point by its final offset along its normal. Positions are done first, in their
  // own pass, because the heading recomputed below for point i depends on point i-1 and i+1's
  // new positions — those need to already be final before any heading is computed.
  nav_msgs::msg::Path deformed = path;
  for (size_t i = 0; i < n; ++i) {
    deformed.poses[i].pose.position.x = path.poses[i].pose.position.x + offset[i] * normal[i].first;
    deformed.poses[i].pose.position.y = path.poses[i].pose.position.y + offset[i] * normal[i].second;
  }

  // The original heading no longer matches the new tangent direction once points have moved,
  // so recompute it from the new positions: forward difference at the first point, backward
  // difference at the last, and a central difference (using both neighbours) everywhere else.
  for (size_t i = 0; i < n; ++i) {
    double yaw;
    if (i == 0) {
      const auto & a = deformed.poses[0].pose.position;
      const auto & b = deformed.poses[1].pose.position;
      yaw = std::atan2(b.y - a.y, b.x - a.x);
    } else if (i == n - 1) {
      const auto & a = deformed.poses[i - 1].pose.position;
      const auto & b = deformed.poses[i].pose.position;
      yaw = std::atan2(b.y - a.y, b.x - a.x);
    } else {
      const auto & a = deformed.poses[i - 1].pose.position;
      const auto & b = deformed.poses[i + 1].pose.position;
      yaw = std::atan2(b.y - a.y, b.x - a.x);
    }
    deformed.poses[i].pose.orientation = quaternion_from_yaw(yaw);
  }

  return deformed;
}

int8_t TrajectoryCore::get_max_footprint_cost(
  double x, double y, double yaw, const nav_msgs::msg::OccupancyGrid & costmap) const
{
  double map_ox = costmap.info.origin.position.x;
  double map_oy = costmap.info.origin.position.y;
  double res = costmap.info.resolution;
  int w = costmap.info.width;
  int h = costmap.info.height;

  auto get_cost = [&](double wx, double wy) -> int8_t {
    int cx = static_cast<int>((wx - map_ox) / res);
    int cy = static_cast<int>((wy - map_oy) / res);
    if (cx < 0 || cx >= w || cy < 0 || cy >= h) return 0;
    return costmap.data[cy * w + cx];
  };

  double cos_t = std::cos(yaw);
  double sin_t = std::sin(yaw);

  auto footprint_cost = [&](double lx, double ly) -> int8_t {
    double wx = x + lx * cos_t - ly * sin_t;
    double wy = y + lx * sin_t + ly * cos_t;
    return get_cost(wx, wy);
  };

  int8_t max_cost = 0;

  // Sample the four corners
  max_cost = std::max(max_cost, footprint_cost(config_.footprint_x_max, config_.footprint_y_max));
  max_cost = std::max(max_cost, footprint_cost(config_.footprint_x_max, config_.footprint_y_min));
  max_cost = std::max(max_cost, footprint_cost(config_.footprint_x_min, config_.footprint_y_max));
  max_cost = std::max(max_cost, footprint_cost(config_.footprint_x_min, config_.footprint_y_min));

  // Sample along the front edge
  for (double ly = config_.footprint_y_min; ly <= config_.footprint_y_max + 1e-9; ly += res) {
    double clamped_ly = std::min(ly, config_.footprint_y_max);
    max_cost = std::max(max_cost, footprint_cost(config_.footprint_x_max, clamped_ly));
  }

  return max_cost;
}

}  // namespace trajectory_planner
