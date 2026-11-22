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
#include <limits>
#include <string>
#include <vector>

#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace fake_planner
{

/** @brief Geometry and frame settings for the published trajectory window. */
struct FakePlannerConfig
{
  double horizon_m{35.0};  ///< Path distance ahead of the vehicle, in metres.
  double trail_m{2.0};  ///< Path distance behind the vehicle, in metres.
  double search_ahead_m{400.0};  ///< Maximum forward search distance, in metres.
  std::string frame_id{"odom"};  ///< Frame shared by odometry and the trajectory.
};

/** @brief Expands maneuver JSON and selects a rolling trajectory window. */
class FakePlannerCore
{
public:
  /**
   * @brief Construct the trajectory helper.
   * @param config Window geometry and output frame.
   */
  explicit FakePlannerCore(const FakePlannerConfig & config);

  /**
   * @brief Load a maneuver from a JSON file.
   * @param path Path to the maneuver file.
   * @param error Receives the reason when loading fails.
   * @return True when the maneuver was loaded.
   */
  bool loadManeuver(const std::string & path, std::string & error);

  /**
   * @brief Load a maneuver from JSON text.
   * @param json_text Maneuver definition with an ordered segment list.
   * @param error Receives the reason when parsing or validation fails.
   * @return True when the maneuver was loaded.
   */
  bool loadManeuverJson(const std::string & json_text, std::string & error);

  /**
   * @brief Transform loaded waypoints into the output frame.
   * @param x Anchor position along the frame's x axis, in metres.
   * @param y Anchor position along the frame's y axis, in metres.
   * @param yaw Anchor heading, in radians.
   */
  void anchor(double x, double y, double yaw);

  /**
   * @brief Select the window around the vehicle after anchoring.
   * @param veh_x Vehicle x position in the output frame, in metres.
   * @param veh_y Vehicle y position in the output frame, in metres.
   */
  void updateWindow(double veh_x, double veh_y);

  /** @brief Restart window matching while keeping the anchored geometry. */
  void rewind();

  /** @brief Clear the anchor while keeping the loaded maneuver for another run. */
  void clear();

  /** @brief Get the current controller window. @return Selected trajectory. */
  const wato_trajectory_msgs::msg::Trajectory & window() const;

  /** @brief Get the full anchored path. @return Reference used for preview and scoring. */
  const wato_trajectory_msgs::msg::Trajectory & reference() const;

  /** @brief Count loaded waypoints. @return Number of uniformly sampled points. */
  std::size_t waypointCount() const;

  /** @brief Check whether the maneuver loops. @return True for a closed path. */
  bool closed() const;

  /** @brief Check for an authored start pose. @return True for absolute geometry. */
  bool hasAbsoluteStart() const;

  /**
   * @brief Measure remaining path distance to the stop line.
   * @return Metres remaining, or infinity before anchoring and on closed paths.
   */
  double distanceToEnd() const;

  /** @brief Measure the full path. @return Arc length including the stop pad, in metres. */
  double pathLength() const;

  /** @brief Measure tracking deviation. @return Distance to the matched point, in metres. */
  double distanceToPath() const;

  /** @brief Check whether the path is anchored. @return True when a trajectory is available. */
  bool ready() const;

private:
  /**
   * @brief Find the nearest point within the forward search distance.
   * @param veh_x Vehicle x position, in metres.
   * @param veh_y Vehicle y position, in metres.
   * @param match_distance Receives distance to the selected point, in metres.
   * @return Index into the anchored trajectory.
   */
  std::size_t nearestIndex(double veh_x, double veh_y, double & match_distance) const;

  // Configuration
  FakePlannerConfig config_;
  bool closed_{false};
  bool has_absolute_start_{false};
  double spacing_{1.0};

  // Loaded geometry and speed profile
  std::vector<double> wp_x_;
  std::vector<double> wp_y_;
  std::vector<double> wp_speed_;
  std::vector<double> cum_s_;
  std::size_t stop_index_{0};

  // Anchored trajectory and current match
  wato_trajectory_msgs::msg::Trajectory full_traj_;
  wato_trajectory_msgs::msg::Trajectory window_;
  bool ready_{false};
  std::size_t window_start_{0};
  double match_distance_{std::numeric_limits<double>::infinity()};
};

}  // namespace fake_planner
