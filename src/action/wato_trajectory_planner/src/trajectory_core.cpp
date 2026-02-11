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

#include "wato_trajectory_planner/trajectory_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace wato_trajectory_planner
{

TrajectoryCore::TrajectoryCore(const TrajectoryConfig & config)
: config_(config)
{}

wato_trajectory_msgs::msg::Trajectory TrajectoryCore::compute_trajectory(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap, double current_speed)
{
  // Creates a dummy trajectory for now
  wato_trajectory_msgs::msg::Trajectory trajectory;
  trajectory.header = path.header;

  // TODO: Implement full logic
  (void)costmap;
  (void)current_speed;

  for (const auto & pose : path.poses) {
    wato_trajectory_msgs::msg::TrajectoryPoint point;
    point.pose = pose.pose;
    point.max_speed = config_.max_speed;
    trajectory.points.push_back(point);
  }

  return trajectory;
}

std::optional<double> TrajectoryCore::find_first_collision(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap)
{
  // TODO: Implement collision checking
  (void)path;
  (void)costmap;
  return std::nullopt;
}

}  // namespace wato_trajectory_planner
