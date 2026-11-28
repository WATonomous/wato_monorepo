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
#include <limits>
#include <utility>
#include <vector>

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

  // Geometry pipeline: resample -> smooth (remove map kinks) -> obstacle deformation.
  // With both stages disabled, the legacy geometry and waypoint count are preserved.
  nav_msgs::msg::Path planning_path = path;
  if (config_.path_smoothing_enabled) {
    planning_path = smooth_path(path);
  } else if (config_.elastic_band_enabled) {
    planning_path = resample_path(path);
  }
  if (config_.elastic_band_enabled) {
    if (auto deformed = elastic_band(planning_path, costmap)) {
      planning_path = std::move(*deformed);
    }
  }

  std::optional<double> obstacle_dist = find_first_collision(planning_path, costmap);
  const bool use_windowed_curvature = config_.curvature_window > 0.0;
  const std::vector<double> curvatures =
    use_windowed_curvature ? compute_curvatures(planning_path) : std::vector<double>{};

  auto yaw_from_pose = [](const geometry_msgs::msg::Pose & p) {
    const auto & q = p.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };

  double effective_max_speed = std::max(0.0, limit_speed);
  double dist_along_path = 0.0;
  double prev_speed = current_speed;
  auto prev_pos = planning_path.poses.front().pose.position;

  for (size_t i = 0; i < planning_path.poses.size(); ++i) {
    const auto & pos = planning_path.poses[i].pose.position;

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

    if (use_windowed_curvature) {
      const double kappa = curvatures[i];
      if (kappa > 1e-6 && target_speed * target_speed * kappa > config_.max_lateral_accel) {
        target_speed = std::sqrt(config_.max_lateral_accel / kappa);
      }
    } else if (i > 0 && segment_dist > 1e-9) {
      double yaw_curr = yaw_from_pose(planning_path.poses[i].pose);
      double yaw_prev = yaw_from_pose(planning_path.poses[i - 1].pose);
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
    double yaw = yaw_from_pose(planning_path.poses[i].pose);
    int8_t cost = get_max_footprint_cost(pos.x, pos.y, yaw, costmap);
    if (cost > 0 && cost < LETHAL_COST) {
      double cost_scale = 1.0 - static_cast<double>(cost) / LETHAL_COST;
      target_speed *= cost_scale;
    }

    target_speed = std::clamp(target_speed, 0.0, effective_max_speed);
    prev_speed = target_speed;

    wato_trajectory_msgs::msg::TrajectoryPoint point;
    point.pose = planning_path.poses[i].pose;
    point.max_speed = target_speed;
    trajectory.points.push_back(point);
  }

  // Backward pass: propagate speed limits backward so the vehicle decelerates
  // in advance of curves, obstacles, and stops instead of braking suddenly.
  if (trajectory.points.size() >= 2) {
    for (int i = static_cast<int>(trajectory.points.size()) - 2; i >= 0; --i) {
      double dx = planning_path.poses[i + 1].pose.position.x - planning_path.poses[i].pose.position.x;
      double dy = planning_path.poses[i + 1].pose.position.y - planning_path.poses[i].pose.position.y;
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
  return find_first_collision_with_margin(path, costmap, 0.0);
}

nav_msgs::msg::Path TrajectoryCore::resample_path(const nav_msgs::msg::Path & path) const
{
  return resample_path(path, config_.interpolation_resolution);
}

nav_msgs::msg::Path TrajectoryCore::resample_path(const nav_msgs::msg::Path & path, double spacing) const
{
  nav_msgs::msg::Path resampled;
  resampled.header = path.header;
  if (path.poses.empty()) {
    return resampled;
  }
  if (path.poses.size() == 1 || spacing <= 0.0) {
    resampled = path;
    update_orientations(resampled);
    return resampled;
  }

  std::vector<double> cumulative(path.poses.size(), 0.0);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & previous = path.poses[i - 1].pose.position;
    const auto & current = path.poses[i].pose.position;
    cumulative[i] = cumulative[i - 1] + std::hypot(current.x - previous.x, current.y - previous.y);
  }

  const double total_length = cumulative.back();
  if (total_length <= 1e-9) {
    resampled.poses.push_back(path.poses.front());
    if (path.poses.size() > 1) {
      resampled.poses.push_back(path.poses.back());
    }
    update_orientations(resampled);
    return resampled;
  }

  size_t segment = 1;
  auto append_at_distance = [&](double distance) {
    while (segment + 1 < path.poses.size() && cumulative[segment] < distance) {
      ++segment;
    }
    const double segment_start = cumulative[segment - 1];
    const double segment_length = cumulative[segment] - segment_start;
    const double ratio = segment_length > 1e-9 ? (distance - segment_start) / segment_length : 0.0;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    const auto & start = path.poses[segment - 1].pose.position;
    const auto & end = path.poses[segment].pose.position;
    pose.pose.position.x = start.x + ratio * (end.x - start.x);
    pose.pose.position.y = start.y + ratio * (end.y - start.y);
    pose.pose.position.z = start.z + ratio * (end.z - start.z);
    resampled.poses.push_back(std::move(pose));
  };

  resampled.poses.push_back(path.poses.front());
  for (double distance = spacing; distance < total_length - 1e-9; distance += spacing) {
    append_at_distance(distance);
  }
  resampled.poses.push_back(path.poses.back());
  update_orientations(resampled);
  return resampled;
}

nav_msgs::msg::Path TrajectoryCore::smooth_path(const nav_msgs::msg::Path & path) const
{
  // Holding two points fixed at each end keeps the start tangent (near the ego vehicle) and the
  // end tangent unchanged, and every free point has a full second-difference stencil.
  constexpr size_t kFixedEndPoints = 2;

  const nav_msgs::msg::Path band_path = resample_path(path, config_.smoothing_spacing);
  const size_t n = band_path.poses.size();
  if (config_.smoothing_spacing <= 0.0 || n < 2 * kFixedEndPoints + 1) {
    return resample_path(path);
  }

  struct Vec2
  {
    double x;
    double y;
  };

  // Original band points and their unit normals (left-positive). Points move only along these
  // fixed normals, so the band cannot bunch up or slide along the path.
  std::vector<Vec2> origin(n);
  std::vector<Vec2> normal(n);
  for (size_t i = 0; i < n; ++i) {
    origin[i] = {band_path.poses[i].pose.position.x, band_path.poses[i].pose.position.y};
  }
  for (size_t i = 0; i < n; ++i) {
    const Vec2 & before = origin[i == 0 ? 0 : i - 1];
    const Vec2 & after = origin[i + 1 < n ? i + 1 : i];
    const double dx = after.x - before.x;
    const double dy = after.y - before.y;
    const double len = std::hypot(dx, dy);
    normal[i] = len > 1e-9 ? Vec2{-dy / len, dx / len} : Vec2{0.0, 1.0};
  }

  // Projected gradient descent on E(d) = sum |L_i|^2 + w_a * sum d_i^2, where
  // p_i = origin_i + d_i * normal_i and L_i = p[i-1] - 2 p[i] + p[i+1].
  // dE/dp_i = 2 * (L[i-1] - 2 L[i] + L[i+1]) (the discrete bilaplacian), so the step on d_i is
  // its projection onto normal_i. The bilaplacian's largest eigenvalue is 16, so step <= 1/8.
  const double step = std::clamp(config_.smoothing_step, 0.0, 0.125);
  std::vector<double> offset(n, 0.0);
  std::vector<Vec2> p(origin);
  std::vector<Vec2> lap(n, {0.0, 0.0});
  for (int iter = 0; iter < config_.smoothing_iterations; ++iter) {
    for (size_t i = 1; i + 1 < n; ++i) {
      lap[i] = {p[i - 1].x - 2.0 * p[i].x + p[i + 1].x, p[i - 1].y - 2.0 * p[i].y + p[i + 1].y};
    }
    // lap[0] and lap[n-1] stay zero: the endpoints are fixed, so they contribute no bending.

    double max_move = 0.0;
    for (size_t i = kFixedEndPoints; i + kFixedEndPoints < n; ++i) {
      const double bx = lap[i - 1].x - 2.0 * lap[i].x + lap[i + 1].x;
      const double by = lap[i - 1].y - 2.0 * lap[i].y + lap[i + 1].y;
      const double grad = bx * normal[i].x + by * normal[i].y + config_.smoothing_anchor_weight * offset[i];
      const double updated =
        std::clamp(offset[i] - step * grad, -config_.smoothing_max_deviation, config_.smoothing_max_deviation);
      max_move = std::max(max_move, std::fabs(updated - offset[i]));
      offset[i] = updated;
    }
    for (size_t i = kFixedEndPoints; i + kFixedEndPoints < n; ++i) {
      p[i] = {origin[i].x + offset[i] * normal[i].x, origin[i].y + offset[i] * normal[i].y};
    }
    if (max_move < config_.smoothing_tolerance) {
      break;
    }
  }

  // Densify the relaxed band with a uniform Catmull-Rom spline (C1: continuous heading), using
  // reflected phantom points at the ends, then resample at the planning resolution.
  auto point_at = [&](long i) -> Vec2 {
    if (i < 0) {
      return {2.0 * p[0].x - p[1].x, 2.0 * p[0].y - p[1].y};
    }
    if (i >= static_cast<long>(n)) {
      return {2.0 * p[n - 1].x - p[n - 2].x, 2.0 * p[n - 1].y - p[n - 2].y};
    }
    return p[static_cast<size_t>(i)];
  };

  const double dense_spacing = config_.interpolation_resolution > 0.0
                                 ? std::min(config_.interpolation_resolution, config_.smoothing_spacing)
                                 : config_.smoothing_spacing;
  nav_msgs::msg::Path dense;
  dense.header = path.header;
  auto push_point = [&](const Vec2 & v) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = v.x;
    pose.pose.position.y = v.y;
    dense.poses.push_back(std::move(pose));
  };
  for (size_t seg = 0; seg + 1 < n; ++seg) {
    const long s = static_cast<long>(seg);
    const Vec2 p0 = point_at(s - 1);
    const Vec2 p1 = point_at(s);
    const Vec2 p2 = point_at(s + 1);
    const Vec2 p3 = point_at(s + 2);
    const double seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);
    // Oversample so the final linear resample sees a curve, not chords.
    const int samples = std::max(1, static_cast<int>(std::ceil(4.0 * seg_len / dense_spacing)));
    for (int k = 0; k < samples; ++k) {
      const double t = static_cast<double>(k) / samples;
      const double t2 = t * t;
      const double t3 = t2 * t;
      auto blend = [&](double a, double b, double c, double d) {
        return 0.5 * ((2.0 * b) + (-a + c) * t + (2.0 * a - 5.0 * b + 4.0 * c - d) * t2 + (-a + 3.0 * b - 3.0 * c + d) * t3);
      };
      push_point({blend(p0.x, p1.x, p2.x, p3.x), blend(p0.y, p1.y, p2.y, p3.y)});
    }
  }
  push_point(p[n - 1]);

  // The band ends coincide with the original path ends (resample_path preserves endpoints).
  dense.poses.front().pose.position.z = path.poses.front().pose.position.z;
  dense.poses.back().pose.position.z = path.poses.back().pose.position.z;
  return resample_path(dense);
}

std::vector<double> TrajectoryCore::compute_curvatures(const nav_msgs::msg::Path & path) const
{
  const size_t n = path.poses.size();
  std::vector<double> curvatures(n, 0.0);
  if (n < 3) {
    return curvatures;
  }

  std::vector<double> arc(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    arc[i] = arc[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }

  const double window = config_.curvature_window;
  size_t back = 0;
  size_t ahead = 0;
  for (size_t i = 0; i < n; ++i) {
    // back: last index at least `window` behind i (or 0); ahead: first index at least `window`
    // ahead of i (or n-1). Both move monotonically with i.
    while (back + 1 < i && arc[i] - arc[back + 1] >= window) {
      ++back;
    }
    if (ahead < i) {
      ahead = i;
    }
    while (ahead + 1 < n && arc[ahead] - arc[i] < window) {
      ++ahead;
    }
    size_t j = std::min(back, i);
    size_t k = ahead;
    // Near the ends the window is one-sided; fall back to the nearest available neighbours.
    if (j == i && i > 0) {
      j = i - 1;
    }
    if (k == i && i + 1 < n) {
      k = i + 1;
    }
    if (j == i || k == i) {
      continue;
    }

    const auto & a = path.poses[j].pose.position;
    const auto & b = path.poses[i].pose.position;
    const auto & c = path.poses[k].pose.position;
    const double ab = std::hypot(b.x - a.x, b.y - a.y);
    const double bc = std::hypot(c.x - b.x, c.y - b.y);
    const double ca = std::hypot(a.x - c.x, a.y - c.y);
    const double denom = ab * bc * ca;
    if (denom < 1e-12) {
      continue;
    }
    const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    curvatures[i] = 2.0 * std::fabs(cross) / denom;
  }
  // The endpoints have no neighbour on one side; reuse the adjacent estimate.
  curvatures.front() = curvatures[1];
  curvatures.back() = curvatures[n - 2];
  return curvatures;
}

void TrajectoryCore::update_orientations(nav_msgs::msg::Path & path) const
{
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = 1.0;
    double dy = 0.0;
    if (path.poses.size() > 1) {
      const size_t previous = i == 0 ? 0 : i - 1;
      const size_t next = i + 1 < path.poses.size() ? i + 1 : i;
      dx = path.poses[next].pose.position.x - path.poses[previous].pose.position.x;
      dy = path.poses[next].pose.position.y - path.poses[previous].pose.position.y;
      if (std::hypot(dx, dy) <= 1e-9) {
        dx = 1.0;
        dy = 0.0;
      }
    }
    const double yaw = std::atan2(dy, dx);
    auto & orientation = path.poses[i].pose.orientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = std::sin(yaw * 0.5);
    orientation.w = std::cos(yaw * 0.5);
  }
}

std::optional<nav_msgs::msg::Path> TrajectoryCore::elastic_band(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap) const
{
  nav_msgs::msg::Path deformed = resample_path(path);
  if (deformed.poses.empty()) {
    return deformed;
  }

  const size_t point_count = deformed.poses.size();
  std::vector<double> distances(point_count, 0.0);
  std::vector<double> yaws(point_count, 0.0);
  std::vector<double> normals_x(point_count, 0.0);
  std::vector<double> normals_y(point_count, 1.0);
  std::vector<bool> colliding(point_count, false);

  for (size_t i = 1; i < point_count; ++i) {
    const auto & previous = deformed.poses[i - 1].pose.position;
    const auto & current = deformed.poses[i].pose.position;
    distances[i] = distances[i - 1] + std::hypot(current.x - previous.x, current.y - previous.y);
  }
  for (size_t i = 0; i < point_count; ++i) {
    const size_t previous = i == 0 ? 0 : i - 1;
    const size_t next = i + 1 < point_count ? i + 1 : i;
    const auto & before = deformed.poses[previous].pose.position;
    const auto & after = deformed.poses[next].pose.position;
    yaws[i] = std::atan2(after.y - before.y, after.x - before.x);
    normals_x[i] = -std::sin(yaws[i]);
    normals_y[i] = std::cos(yaws[i]);
    const auto & position = deformed.poses[i].pose.position;
    colliding[i] = get_max_footprint_cost(position.x, position.y, yaws[i], costmap) >= LETHAL_COST;
  }

  struct CollisionCluster
  {
    size_t start;
    size_t end;
  };

  std::vector<CollisionCluster> clusters;
  for (size_t i = 0; i < point_count;) {
    if (!colliding[i]) {
      ++i;
      continue;
    }
    const size_t start = i;
    while (i + 1 < point_count && colliding[i + 1]) {
      ++i;
    }
    clusters.push_back({start, i});
    ++i;
  }

  if (clusters.empty()) {
    if (find_first_collision_with_margin(deformed, costmap, config_.eb_clearance_margin)) {
      return std::nullopt;
    }
    return deformed;
  }

  std::vector<CollisionCluster> merged;
  for (const auto & cluster : clusters) {
    if (
      !merged.empty() &&
      distances[cluster.start] - distances[merged.back().end] <= 2.0 * config_.eb_transition_distance + 1e-9)
    {
      merged.back().end = cluster.end;
    } else {
      merged.push_back(cluster);
    }
  }

  if (config_.eb_lateral_search_step <= 0.0 || config_.eb_max_deviation <= 0.0 || config_.eb_transition_distance < 0.0)
  {
    return std::nullopt;
  }

  std::vector<double> offsets(point_count, 0.0);
  const double total_length = distances.back();
  for (const auto & cluster : merged) {
    const double start_distance = distances[cluster.start];
    const double end_distance = distances[cluster.end];
    if (
      start_distance + 1e-9 < config_.eb_transition_distance ||
      total_length - end_distance + 1e-9 < config_.eb_transition_distance)
    {
      return std::nullopt;
    }

    auto smallest_valid_offset = [&](double side_sign) -> std::optional<double> {
      const int steps = static_cast<int>(std::floor(config_.eb_max_deviation / config_.eb_lateral_search_step + 1e-9));
      for (int step = 1; step <= steps; ++step) {
        const double deviation = step * config_.eb_lateral_search_step;
        bool clear = true;
        for (size_t i = cluster.start; i <= cluster.end; ++i) {
          const auto & position = deformed.poses[i].pose.position;
          const double shifted_x = position.x + side_sign * deviation * normals_x[i];
          const double shifted_y = position.y + side_sign * deviation * normals_y[i];
          if (
            get_max_footprint_cost(shifted_x, shifted_y, yaws[i], costmap, config_.eb_clearance_margin) >= LETHAL_COST)
          {
            clear = false;
            break;
          }
        }
        if (clear) {
          return deviation;
        }
      }
      return std::nullopt;
    };

    const auto left = smallest_valid_offset(1.0);
    const auto right = smallest_valid_offset(-1.0);
    if (!left && !right) {
      return std::nullopt;
    }

    double side = 1.0;
    double deviation = 0.0;
    if (left && right) {
      if (std::fabs(*left - *right) <= 1e-9) {
        side = config_.eb_preferred_side_sign < 0.0 ? -1.0 : 1.0;
        deviation = *left;
      } else if (*left < *right) {
        deviation = *left;
      } else {
        side = -1.0;
        deviation = *right;
      }
    } else if (left) {
      deviation = *left;
    } else {
      side = -1.0;
      deviation = *right;
    }

    const double ramp_in_start = start_distance - config_.eb_transition_distance;
    const double ramp_out_end = end_distance + config_.eb_transition_distance;
    auto smoothstep = [](double value) { return value * value * (3.0 - 2.0 * value); };
    for (size_t i = 0; i < point_count; ++i) {
      double scale = 0.0;
      if (distances[i] >= start_distance && distances[i] <= end_distance) {
        scale = 1.0;
      } else if (distances[i] > ramp_in_start && distances[i] < start_distance && config_.eb_transition_distance > 0.0)
      {
        scale = smoothstep((distances[i] - ramp_in_start) / config_.eb_transition_distance);
      } else if (distances[i] > end_distance && distances[i] < ramp_out_end && config_.eb_transition_distance > 0.0) {
        scale = smoothstep((ramp_out_end - distances[i]) / config_.eb_transition_distance);
      }
      if (scale > 0.0) {
        offsets[i] = side * deviation * scale;
      }
    }
  }

  for (size_t i = 0; i < point_count; ++i) {
    deformed.poses[i].pose.position.x += offsets[i] * normals_x[i];
    deformed.poses[i].pose.position.y += offsets[i] * normals_y[i];
  }
  update_orientations(deformed);

  if (find_first_collision_with_margin(deformed, costmap, config_.eb_clearance_margin)) {
    return std::nullopt;
  }
  return deformed;
}

int8_t TrajectoryCore::get_max_footprint_cost(
  double x, double y, double yaw, const nav_msgs::msg::OccupancyGrid & costmap, double lateral_margin) const
{
  const double resolution = costmap.info.resolution;
  const int width = static_cast<int>(costmap.info.width);
  const int height = static_cast<int>(costmap.info.height);
  if (resolution <= 0.0 || width <= 0 || height <= 0 || costmap.data.empty()) {
    return 0;
  }

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double half_cell_diagonal = resolution / std::sqrt(2.0);
  const double local_x_min = config_.footprint_x_min - half_cell_diagonal;
  const double local_x_max = config_.footprint_x_max + half_cell_diagonal;
  const double local_y_min = config_.footprint_y_min - lateral_margin - half_cell_diagonal;
  const double local_y_max = config_.footprint_y_max + lateral_margin + half_cell_diagonal;

  const double radius = std::hypot(
    std::max(std::fabs(local_x_min), std::fabs(local_x_max)), std::max(std::fabs(local_y_min), std::fabs(local_y_max)));
  const double origin_x = costmap.info.origin.position.x;
  const double origin_y = costmap.info.origin.position.y;
  const int min_col = std::max(0, static_cast<int>(std::floor((x - radius - origin_x) / resolution)));
  const int max_col = std::min(width - 1, static_cast<int>(std::floor((x + radius - origin_x) / resolution)));
  const int min_row = std::max(0, static_cast<int>(std::floor((y - radius - origin_y) / resolution)));
  const int max_row = std::min(height - 1, static_cast<int>(std::floor((y + radius - origin_y) / resolution)));

  int8_t max_cost = 0;
  for (int row = min_row; row <= max_row; ++row) {
    for (int col = min_col; col <= max_col; ++col) {
      const double cell_x = origin_x + (static_cast<double>(col) + 0.5) * resolution;
      const double cell_y = origin_y + (static_cast<double>(row) + 0.5) * resolution;
      const double dx = cell_x - x;
      const double dy = cell_y - y;
      const double local_x = dx * cos_yaw + dy * sin_yaw;
      const double local_y = -dx * sin_yaw + dy * cos_yaw;
      if (local_x >= local_x_min && local_x <= local_x_max && local_y >= local_y_min && local_y <= local_y_max) {
        max_cost = std::max(max_cost, costmap.data[static_cast<size_t>(row) * costmap.info.width + col]);
      }
    }
  }
  return max_cost;
}

std::optional<double> TrajectoryCore::find_first_collision_with_margin(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap, double lateral_margin) const
{
  if (path.poses.empty()) {
    return std::nullopt;
  }

  auto yaw_from_pose = [](const geometry_msgs::msg::Pose & pose) {
    const auto & q = pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };
  if (path.poses.size() == 1) {
    const auto & pose = path.poses.front().pose;
    return get_max_footprint_cost(pose.position.x, pose.position.y, yaw_from_pose(pose), costmap, lateral_margin) >=
               LETHAL_COST
             ? std::optional<double>(0.0)
             : std::nullopt;
  }

  const double sampling_resolution = config_.interpolation_resolution > 0.0
                                       ? config_.interpolation_resolution
                                       : std::max(0.01, static_cast<double>(costmap.info.resolution));
  double distance_along_path = 0.0;
  double last_yaw = 0.0;
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & start = path.poses[i].pose.position;
    const auto & end = path.poses[i + 1].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double segment_length = std::hypot(dx, dy);
    last_yaw = segment_length > 1e-9 ? std::atan2(dy, dx) : last_yaw;
    const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / sampling_resolution)));
    for (int step = 0; step < steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      const double sample_x = start.x + ratio * dx;
      const double sample_y = start.y + ratio * dy;
      if (get_max_footprint_cost(sample_x, sample_y, last_yaw, costmap, lateral_margin) >= LETHAL_COST) {
        return distance_along_path + ratio * segment_length;
      }
    }
    distance_along_path += segment_length;
  }

  const auto & final_position = path.poses.back().pose.position;
  if (get_max_footprint_cost(final_position.x, final_position.y, last_yaw, costmap, lateral_margin) >= LETHAL_COST) {
    return distance_along_path;
  }
  return std::nullopt;
}

}  // namespace trajectory_planner
