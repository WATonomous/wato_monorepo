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

  if (path.poses.empty()) {
    return trajectory;
  }

  // Laterally deform the path around obstacles before velocity profiling. The deformed path
  // (or the original path when disabled) is used for everything downstream: collision checks,
  // curvature-based speed limiting, footprint cost scaling, and the output poses. If the band
  // cannot find a clear deformation, find_first_collision below still finds the obstacle on the
  // deformed path and the existing stop-before-collision logic kicks in as a fallback.
  const nav_msgs::msg::Path deformed_path = config_.elastic_band_enabled ? elastic_band(path, costmap) : path;

  std::optional<double> obstacle_dist = find_first_collision(deformed_path, costmap);

  auto yaw_from_pose = [](const geometry_msgs::msg::Pose & p) {
    const auto & q = p.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };

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

nav_msgs::msg::Path TrajectoryCore::elastic_band(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap) const
{
  nav_msgs::msg::Path deformed = path;

  if (path.poses.size() < 3) {
    return deformed;
  }

  struct Point2D
  {
    double x;
    double y;
  };

  const size_t n = path.poses.size();
  std::vector<Point2D> orig(n);
  std::vector<Point2D> band(n);
  for (size_t i = 0; i < n; ++i) {
    orig[i] = {path.poses[i].pose.position.x, path.poses[i].pose.position.y};
    band[i] = orig[i];
  }

  const double map_ox = costmap.info.origin.position.x;
  const double map_oy = costmap.info.origin.position.y;
  const double res = costmap.info.resolution;
  const int w = static_cast<int>(costmap.info.width);
  const int h = static_cast<int>(costmap.info.height);
  const int radius_cells =
    res > 0.0 ? std::max(0, static_cast<int>(std::ceil(config_.eb_influence_radius / res))) : 0;

  // Finds the nearest lethal cell to (x, y) by scanning a local window of cells (sized from
  // eb_influence_radius) rather than the whole grid.
  auto nearest_lethal = [&](double x, double y) -> std::optional<Point2D> {
    if (res <= 0.0 || w <= 0 || h <= 0) {
      return std::nullopt;
    }

    int cx = static_cast<int>(std::floor((x - map_ox) / res));
    int cy = static_cast<int>(std::floor((y - map_oy) / res));

    int cx_min = std::max(0, cx - radius_cells);
    int cx_max = std::min(w - 1, cx + radius_cells);
    int cy_min = std::max(0, cy - radius_cells);
    int cy_max = std::min(h - 1, cy + radius_cells);

    bool found = false;
    double best_dist_sq = std::numeric_limits<double>::max();
    Point2D best{0.0, 0.0};

    for (int gy = cy_min; gy <= cy_max; ++gy) {
      for (int gx = cx_min; gx <= cx_max; ++gx) {
        if (costmap.data[gy * w + gx] < LETHAL_COST) {
          continue;
        }
        double ox = map_ox + (gx + 0.5) * res;
        double oy = map_oy + (gy + 0.5) * res;
        double dist_sq = (x - ox) * (x - ox) + (y - oy) * (y - oy);
        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best = {ox, oy};
          found = true;
        }
      }
    }

    return found ? std::optional<Point2D>(best) : std::nullopt;
  };

  for (int iter = 0; iter < config_.eb_max_iterations; ++iter) {
    double max_disp = 0.0;

    for (size_t i = 1; i + 1 < n; ++i) {
      // Smoothing force: standard discrete contraction toward the neighbours' midpoint.
      double fs_x = config_.eb_smooth_weight * (band[i - 1].x + band[i + 1].x - 2.0 * band[i].x);
      double fs_y = config_.eb_smooth_weight * (band[i - 1].y + band[i + 1].y - 2.0 * band[i].y);

      // Obstacle repulsion force: pushes away from the nearest lethal cell within influence range.
      double fo_x = 0.0;
      double fo_y = 0.0;
      auto obstacle = nearest_lethal(band[i].x, band[i].y);
      if (obstacle.has_value()) {
        double dx = band[i].x - obstacle->x;
        double dy = band[i].y - obstacle->y;
        double d = std::hypot(dx, dy);
        if (d < config_.eb_influence_radius) {
          double ux = 0.0;
          double uy = 0.0;
          if (d < 1e-6) {
            // Degenerate case: point sits on top of the obstacle. Fall back to a direction
            // perpendicular to the local path tangent so the point still has somewhere to go.
            double tx = band[i + 1].x - band[i - 1].x;
            double ty = band[i + 1].y - band[i - 1].y;
            double tlen = std::hypot(tx, ty);
            if (tlen < 1e-9) {
              ux = 0.0;
              uy = 1.0;
            } else {
              ux = -ty / tlen;
              uy = tx / tlen;
            }
          } else {
            ux = dx / d;
            uy = dy / d;
          }
          double mag = config_.eb_obstacle_weight * (1.0 - d / config_.eb_influence_radius);
          fo_x = mag * ux;
          fo_y = mag * uy;
        }
      }

      // Anchor force: pulls the point back toward its original (un-deformed) position.
      double fa_x = config_.eb_anchor_weight * (orig[i].x - band[i].x);
      double fa_y = config_.eb_anchor_weight * (orig[i].y - band[i].y);

      double old_x = band[i].x;
      double old_y = band[i].y;

      band[i].x += config_.eb_step_size * (fs_x + fo_x + fa_x);
      band[i].y += config_.eb_step_size * (fs_y + fo_y + fa_y);

      // Clamp the point back onto the max-deviation circle around its original position.
      double dev_x = band[i].x - orig[i].x;
      double dev_y = band[i].y - orig[i].y;
      double dev = std::hypot(dev_x, dev_y);
      if (dev > config_.eb_max_deviation && dev > 1e-9) {
        double scale = config_.eb_max_deviation / dev;
        band[i].x = orig[i].x + dev_x * scale;
        band[i].y = orig[i].y + dev_y * scale;
      }

      max_disp = std::max(max_disp, std::hypot(band[i].x - old_x, band[i].y - old_y));
    }

    if (max_disp < config_.eb_convergence_tol) {
      break;
    }
  }

  for (size_t i = 0; i < n; ++i) {
    deformed.poses[i].pose.position.x = band[i].x;
    deformed.poses[i].pose.position.y = band[i].y;

    double yaw;
    if (i == 0) {
      yaw = std::atan2(band[1].y - band[0].y, band[1].x - band[0].x);
    } else if (i + 1 == n) {
      yaw = std::atan2(band[i].y - band[i - 1].y, band[i].x - band[i - 1].x);
    } else {
      yaw = std::atan2(band[i + 1].y - band[i - 1].y, band[i + 1].x - band[i - 1].x);
    }

    // Yaw-only quaternion; position/z are otherwise untouched.
    deformed.poses[i].pose.orientation.x = 0.0;
    deformed.poses[i].pose.orientation.y = 0.0;
    deformed.poses[i].pose.orientation.z = std::sin(yaw * 0.5);
    deformed.poses[i].pose.orientation.w = std::cos(yaw * 0.5);
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
