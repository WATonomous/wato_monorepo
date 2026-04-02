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

// For pure C++ logging without node dependency, we can use iostream or rclcpp::Logger if passed.
// For now, let's stick to safe defaults.

namespace trajectory_planner
{

static constexpr int8_t LETHAL_COST = 100;

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

  std::optional<double> obstacle_dist = find_first_collision(path, costmap);

  if (path.poses.empty()) {
    return trajectory;
  }

  auto yaw_from_pose = [](const geometry_msgs::msg::Pose & p) {
    const auto & q = p.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };

  double effective_max_speed = std::max(0.0, limit_speed);
  double dist_along_path = 0.0;
  double prev_speed = current_speed;
  auto prev_pos = path.poses.front().pose.position;

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & pos = path.poses[i].pose.position;

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
      double yaw_curr = yaw_from_pose(path.poses[i].pose);
      double yaw_prev = yaw_from_pose(path.poses[i - 1].pose);
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
    double yaw = yaw_from_pose(path.poses[i].pose);
    int8_t cost = get_max_footprint_cost(pos.x, pos.y, yaw, costmap);
    if (cost > 0 && cost < LETHAL_COST) {
      double cost_scale = 1.0 - static_cast<double>(cost) / LETHAL_COST;
      target_speed *= cost_scale;
    }

    target_speed = std::clamp(target_speed, 0.0, effective_max_speed);
    prev_speed = target_speed;

    wato_trajectory_msgs::msg::TrajectoryPoint point;
    point.pose = path.poses[i].pose;
    point.max_speed = target_speed;
    trajectory.points.push_back(point);
  }

  // Backward pass: propagate speed limits backward so the vehicle decelerates
  // in advance of curves, obstacles, and stops instead of braking suddenly.
  if (trajectory.points.size() >= 2) {
    for (int i = static_cast<int>(trajectory.points.size()) - 2; i >= 0; --i) {
      double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
      double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
      double seg = std::hypot(dx, dy);
      if (seg > 1e-9) {
        double next_speed = trajectory.points[i + 1].max_speed;
        double v_max = std::sqrt(next_speed * next_speed + 2.0 * config_.max_lateral_accel * seg);
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
