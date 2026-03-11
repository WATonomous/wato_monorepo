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

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace trajectory_planner
{

/// Cost values sampled from a rotated vehicle footprint at a given pose.
struct FootprintCosts
{
  double avg{0.0};  ///< Mean cost over the full footprint
  double left{0.0};  ///< Mean cost over the left half  (body-frame by > 0)
  double right{0.0};  ///< Mean cost over the right half (body-frame by < 0)
  double max{0.0};  ///< Maximum single cost within the footprint
  double risk{0.0};  ///< Log-sum-exp aggregate cost (C_risk): sensitive to peak values
};

/// Return the costmap value [0, 100] at world (x, y). Returns 0.0 if out of bounds.
double get_costmap_cost(double x, double y, const nav_msgs::msg::OccupancyGrid & costmap);

/// Sample the vehicle footprint rotated to heading `theta` centred at (cx, cy).
/// The footprint bounding box is defined in the body frame:
///   x in [x_min, x_max], y in [y_min, y_max], sampled at `sample_res` spacing.
/// Left half: by > 0 (positive normal direction).  Right half: by < 0.
/// `alpha` is the log-sum-exp risk sensitivity exponent (default 0.08).
FootprintCosts sample_footprint_costs(
  double cx,
  double cy,
  double theta,
  double x_min,
  double x_max,
  double y_min,
  double y_max,
  double sample_res,
  const nav_msgs::msg::OccupancyGrid & costmap,
  double alpha = 0.08);

}  // namespace trajectory_planner
