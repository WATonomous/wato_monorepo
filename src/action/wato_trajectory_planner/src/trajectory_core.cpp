#include "wato_trajectory_planner/trajectory_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
// For pure C++ logging without node dependency, we can use iostream or rclcpp::Logger if passed.
// For now, let's stick to safe defaults.

namespace wato_trajectory_planner
{

TrajectoryCore::TrajectoryCore(const TrajectoryConfig & config)
: config_(config)
{
}

wato_trajectory_msgs::msg::Trajectory TrajectoryCore::compute_trajectory(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap,
  double limit_speed, [[maybe_unused]] double current_speed)
{
  wato_trajectory_msgs::msg::Trajectory trajectory;
  trajectory.header = path.header;

  std::optional<double> obstacle_dist = find_first_collision(path, costmap);

  if (path.poses.empty()) {
      return trajectory;
  }

  // Use the passed limit_speed directly (which is either lane limit or config max_speed)
  double effective_max_speed = limit_speed;
  effective_max_speed = std::max(0.0, effective_max_speed);

  double dist_along_path = 0.0;
  auto prev_pose = path.poses.front().pose.position;

  for (const auto & pose_stamped : path.poses) {
    wato_trajectory_msgs::msg::TrajectoryPoint point;
    point.pose = pose_stamped.pose;

    double segment_dist = std::hypot(
      pose_stamped.pose.position.x - prev_pose.x,
      pose_stamped.pose.position.y - prev_pose.y);
    dist_along_path += segment_dist;
    prev_pose = pose_stamped.pose.position;

    double target_speed = effective_max_speed;

    if (obstacle_dist.has_value()) {
        // Distance remaining from current point to obstacle
        double dist_remaining = *obstacle_dist - dist_along_path;

        if (dist_remaining <= config_.stop_distance) {
            target_speed = 0.0;
        } else if (dist_remaining < config_.safe_distance) {
            // Linear interpolation: d_stop -> 0, d_safe -> effective_max_speed
            double ratio = (dist_remaining - config_.stop_distance) /
                           (config_.safe_distance - config_.stop_distance);
            target_speed = effective_max_speed * ratio;
        }
    }

    // Apply limit
    target_speed = std::max(0.0, std::min(target_speed, effective_max_speed));

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

        // Check footprint radius (simplified as square box)
        double r = config_.footprint_radius;
        
        // Lethal check (100)
        // Check center
        if (get_cost(interp_x, interp_y) >= 100) return total_dist + (s * segment_len);

        // Check corners of bounding box
        if (get_cost(interp_x + r, interp_y + r) >= 100) return total_dist + (s * segment_len);
        if (get_cost(interp_x - r, interp_y + r) >= 100) return total_dist + (s * segment_len);
        if (get_cost(interp_x + r, interp_y - r) >= 100) return total_dist + (s * segment_len);
        if (get_cost(interp_x - r, interp_y - r) >= 100) return total_dist + (s * segment_len);
    }

    total_dist += segment_len;
    prev_pose = curr_pt;
  }

  return std::nullopt;
}

}  // namespace wato_trajectory_planner
