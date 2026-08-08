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

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace fake_planner
{

struct FakePlannerConfig
{
  // Rolling window geometry. The real planner publishes a ~30 m horizon starting just behind the
  // vehicle, replanned as it drives, so the core slides the same window along its maneuver.
  double horizon_m{35.0};
  double trail_m{2.0};
  std::string frame_id{"odom"};
};

// Maneuver expansion and rolling-window slicing for the fake planner: parses the segment list,
// expands it into a uniformly-spaced path, lays that out from an anchor pose, and slices the
// published horizon out of it. Holds no ROS state.
class FakePlannerCore
{
public:
  explicit FakePlannerCore(const FakePlannerConfig & config);

  // Returns false and sets `error` on any I/O, JSON, or schema failure.
  bool loadManeuver(const std::string & path, std::string & error);
  bool loadManeuverJson(const std::string & json_text, std::string & error);

  // Lays the loaded waypoints out from an SE(2) pose in frame_id, building the full trajectory.
  void anchor(double x, double y, double yaw);

  // Slices the rolling window around the vehicle. No-op until anchor() has run.
  void updateWindow(double veh_x, double veh_y);

  // Rewinds the window to the start of the maneuver, keeping the built trajectory.
  void rewind();

  // Drops the built trajectory (the loaded waypoints are kept) so the next anchor() re-lays it.
  void clear();

  const wato_trajectory_msgs::msg::Trajectory & window() const
  {
    return window_;
  }

  // < 2 means the file did not describe a usable path.
  std::size_t waypointCount() const
  {
    return wp_x_.size();
  }

  // From the maneuver's "closed" flag: a closed circuit laps forever (the window wraps past the
  // end), an open maneuver ends in a stop.
  bool closed() const
  {
    return closed_;
  }

  // Set when the maneuver carried a "start" pose: its waypoints are authored on fixed geometry in
  // absolute frame_id coordinates, so anchoring one to the launch pose slides it off the road it
  // was drawn for. The node's `anchoring` parameter defaults off this.
  bool hasAbsoluteStart() const
  {
    return has_absolute_start_;
  }

  // Path distance from the vehicle to the stop line (the last authored waypoint, where the braking
  // profile reaches zero). 0 once at or past it; infinity on a closed circuit and before anchoring.
  double distanceToEnd() const;

  bool ready() const
  {
    return ready_;
  }

private:
  // Index of the full-path point closest to (veh_x, veh_y), searched forward from the last match
  // so a self-intersecting maneuver doesn't snap back to an earlier lap.
  std::size_t nearestIndex(double veh_x, double veh_y) const;

  FakePlannerConfig config_;

  bool closed_{false};
  bool has_absolute_start_{false};

  // Waypoints expanded from the maneuver segments (parallel arrays; relative to the anchor pose).
  std::vector<double> wp_x_;
  std::vector<double> wp_y_;
  std::vector<double> wp_speed_;

  // Arc length at each waypoint and the index of the stop line, both computed once at load:
  // anchoring is a rigid SE(2) transform, so it doesn't change distances along the path.
  std::vector<double> cum_s_;
  std::size_t stop_index_{0};

  wato_trajectory_msgs::msg::Trajectory full_traj_;
  wato_trajectory_msgs::msg::Trajectory window_;
  bool ready_{false};
  std::size_t window_start_{0};
};

}  // namespace fake_planner
