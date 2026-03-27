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

    // if (i > 0 && segment_dist > 1e-9) {
    //   double yaw_curr = yaw_from_pose(path.poses[i].pose);
    //   double yaw_prev = yaw_from_pose(path.poses[i - 1].pose);
    //   double delta_theta = std::fabs(std::remainder(yaw_curr - yaw_prev, 2.0 * M_PI));
    //   if (delta_theta > 1e-6) {
    //     double radius = segment_dist / delta_theta;
    //     if ((target_speed * target_speed) / radius > config_.max_lateral_accel) {
    //       target_speed = std::sqrt(config_.max_lateral_accel * radius);
    //     }
    //   }
    // }

    double tang_accel =
      segment_dist > 0.0 ? (target_speed * target_speed - prev_speed * prev_speed) / (2.0 * segment_dist) : 0.0;

    if (std::fabs(tang_accel) > config_.max_tangential_accel || segment_dist == 0.0) {
      double signed_accel = std::copysign(config_.max_tangential_accel, tang_accel);
      double v_sq = prev_speed * prev_speed + 2.0 * signed_accel * segment_dist;
      target_speed = std::sqrt(std::max(0.0, v_sq));
    }

    if (obstacle_dist.has_value()) {
      double dist_remaining = *obstacle_dist - dist_along_path;
      if (dist_remaining <= config_.stop_distance) {
        target_speed = 0.0;
      } else if (dist_remaining < config_.safe_distance) {
        double ratio = (dist_remaining - config_.stop_distance) / (config_.safe_distance - config_.stop_distance);
        target_speed = std::min(target_speed, effective_max_speed * ratio);
      }
    }

    target_speed = std::clamp(target_speed, 0.0, effective_max_speed);
    prev_speed = target_speed;

    wato_trajectory_msgs::msg::TrajectoryPoint point;
    point.pose = path.poses[i].pose;
    point.max_speed = target_speed;
    trajectory.points.push_back(point);
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

}  // namespace trajectory_planner
