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
  // Rolling window geometry. The real planner never publishes the whole route: the lattice planner
  // emits a ~30 m horizon starting 2 m behind the vehicle, replanned as it drives. Publishing the
  // full maneuver instead makes both the markers and the controller's search look nothing like the
  // real thing, so the core slides the same window along its maneuver.
  double horizon_m{35.0};
  double trail_m{2.0};
  // Frame the trajectory is published in (also the frame the anchor pose is expressed in).
  std::string frame_id{"odom"};
};

/**
 * @brief Maneuver expansion and rolling-window slicing for the fake planner.
 *
 * Owns everything between the maneuver JSON on disk and the trajectory message the node publishes:
 * parsing the segment list, expanding it into a smooth uniformly-spaced path, laying that path out
 * from an anchor pose, and slicing the published horizon out of it as the vehicle drives. Holds no
 * ROS node state, so it can be exercised without a running graph.
 */
class FakePlannerCore
{
public:
  explicit FakePlannerCore(const FakePlannerConfig & config);

  // Reads a maneuver JSON (segment list) off disk. Returns false and sets `error` on any I/O,
  // JSON, or schema failure.
  bool loadManeuver(const std::string & path, std::string & error);
  // Same, from an already-read JSON document; loadManeuver is a thin file-reading wrapper.
  bool loadManeuverJson(const std::string & json_text, std::string & error);

  // Lays the loaded waypoints out from an SE(2) pose in frame_id, building the full trajectory.
  // Until updateWindow runs, the whole maneuver is what window() returns.
  void anchor(double x, double y, double yaw);

  // Slices the rolling window out of the full maneuver around the vehicle's current position.
  // No-op until anchor() has run.
  void updateWindow(double veh_x, double veh_y);

  // Rewinds the window to the start of the maneuver, keeping the built trajectory.
  void rewind();

  // Drops the built trajectory (the loaded waypoints are kept) so the next anchor() re-lays it.
  void clear();

  // The slice of the maneuver to publish this tick.
  const wato_trajectory_msgs::msg::Trajectory & window() const
  {
    return window_;
  }

  // Waypoints expanded from the maneuver; < 2 means the file did not describe a usable path.
  std::size_t waypointCount() const
  {
    return wp_x_.size();
  }

  // Set from the maneuver's "closed" flag: a closed circuit laps forever (the window wraps past
  // the end), an open maneuver ends in a stop.
  bool closed() const
  {
    return closed_;
  }

  // Set when the maneuver carried a "start" pose: its waypoints are authored on fixed geometry in
  // absolute frame_id coordinates, not relative to the vehicle. Anchoring one of those to the
  // launch pose slides it off the road it was drawn for, which is why the node's `anchoring`
  // parameter takes its default decision from here.
  bool hasAbsoluteStart() const
  {
    return has_absolute_start_;
  }

  // Path distance from the vehicle's current place on the maneuver to the stop line -- the last
  // authored waypoint, where the braking profile brings the speed to zero (the trailing stop pad
  // continues past it, see appendStopPad). 0 once the vehicle is at or past it. Infinity on a
  // closed circuit, which has no end, and before anchoring. Lets the node tell "arrived at the
  // end" apart from "still driving", without duplicating the path geometry.
  double distanceToEnd() const;

  // Whether a trajectory has been anchored and is ready to publish.
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

  // Arc length along the maneuver at each waypoint, and the index of the stop line (the first
  // zero-speed waypoint of the terminal pad). Both are computed once at load: anchoring is a
  // rigid SE(2) transform, so it moves the path without changing any distance along it.
  std::vector<double> cum_s_;
  std::size_t stop_index_{0};

  // The whole anchored maneuver (built once the anchor pose is known), and the rolling window of
  // it that actually gets published each tick.
  wato_trajectory_msgs::msg::Trajectory full_traj_;
  wato_trajectory_msgs::msg::Trajectory window_;
  bool ready_{false};
  std::size_t window_start_{0};
};

}  // namespace fake_planner
