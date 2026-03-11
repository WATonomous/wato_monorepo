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

#include "trajectory_planner/elastic_costmap.hpp"

#include <cmath>

namespace trajectory_planner
{

double get_costmap_cost(double x, double y, const nav_msgs::msg::OccupancyGrid & costmap)
{
  double res = costmap.info.resolution;
  int cx = static_cast<int>((x - costmap.info.origin.position.x) / res);
  int cy = static_cast<int>((y - costmap.info.origin.position.y) / res);
  int width = static_cast<int>(costmap.info.width);
  int height = static_cast<int>(costmap.info.height);

  if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
    return 0.0;
  }

  int8_t raw = costmap.data[cy * width + cx];
  // Negative costmap values are "unknown" — treat as free
  return raw < 0 ? 0.0 : static_cast<double>(raw);
}

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
  double alpha)
{
  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);

  double sum_all = 0.0;
  double sum_left = 0.0;
  double sum_right = 0.0;
  double sum_exp = 0.0;
  int count_all = 0;
  int count_left = 0;
  int count_right = 0;
  double max_cost = 0.0;

  // Iterate over body-frame grid
  for (double bx = x_min; bx <= x_max + 1e-6; bx += sample_res) {
    for (double by = y_min; by <= y_max + 1e-6; by += sample_res) {
      // Rotate body-frame sample point into world frame
      double wx = cx + bx * cos_t - by * sin_t;
      double wy = cy + bx * sin_t + by * cos_t;
      double cost = get_costmap_cost(wx, wy, costmap);

      if (cost > max_cost) {
        max_cost = cost;
      }

      sum_all += cost;
      sum_exp += std::exp(alpha * cost);
      count_all++;

      if (by > 0.0) {
        sum_left += cost;
        count_left++;
      } else if (by < 0.0) {
        sum_right += cost;
        count_right++;
      }
    }
  }

  FootprintCosts result;
  result.avg = (count_all > 0) ? sum_all / count_all : 0.0;
  result.left = (count_left > 0) ? sum_left / count_left : 0.0;
  result.right = (count_right > 0) ? sum_right / count_right : 0.0;
  result.max = max_cost;
  result.risk = (count_all > 0) ? (1.0 / alpha) * std::log(sum_exp / count_all) : 0.0;
  return result;
}

}  // namespace trajectory_planner
