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

#include "costmap/costmap_utils.hpp"

#include <algorithm>
#include <cmath>

namespace costmap
{

void markBox(
  nav_msgs::msg::OccupancyGrid & grid, double cx, double cy, double yaw, double half_x, double half_y, int8_t cost)
{
  const auto & info = grid.info;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const double res = info.resolution;
  const int w = static_cast<int>(info.width);
  const int h = static_cast<int>(info.height);

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  // Corners in local frame
  double corners_local[4][2] = {{-half_x, -half_y}, {half_x, -half_y}, {half_x, half_y}, {-half_x, half_y}};

  // Transform corners to grid frame and find bounding box
  double min_gx = 1e9, max_gx = -1e9, min_gy = 1e9, max_gy = -1e9;
  for (auto & c : corners_local) {
    double gx = cx + cos_yaw * c[0] - sin_yaw * c[1];
    double gy = cy + sin_yaw * c[0] + cos_yaw * c[1];
    min_gx = std::min(min_gx, gx);
    max_gx = std::max(max_gx, gx);
    min_gy = std::min(min_gy, gy);
    max_gy = std::max(max_gy, gy);
  }

  int min_col = std::max(0, static_cast<int>((min_gx - ox) / res));
  int max_col = std::min(w - 1, static_cast<int>((max_gx - ox) / res));
  int min_row = std::max(0, static_cast<int>((min_gy - oy) / res));
  int max_row = std::min(h - 1, static_cast<int>((max_gy - oy) / res));

  for (int row = min_row; row <= max_row; ++row) {
    for (int col = min_col; col <= max_col; ++col) {
      double wx = ox + (col + 0.5) * res;
      double wy = oy + (row + 0.5) * res;

      // Transform to box-local frame
      double dx = wx - cx;
      double dy = wy - cy;
      double lx = cos_yaw * dx + sin_yaw * dy;
      double ly = -sin_yaw * dx + cos_yaw * dy;

      if (std::abs(lx) <= half_x && std::abs(ly) <= half_y) {
        int idx = row * w + col;
        grid.data[idx] = std::max(grid.data[idx], cost);
      }
    }
  }
}

void inflateCell(
  nav_msgs::msg::OccupancyGrid & grid, int row, int col, double inflation_m, double cost_decay)
{
  const auto & info = grid.info;
  const double res = info.resolution;
  const int w = static_cast<int>(info.width);
  const int h = static_cast<int>(info.height);

  int inf_cells = static_cast<int>(std::ceil(inflation_m / res));
  for (int dr = -inf_cells; dr <= inf_cells; ++dr) {
    for (int dc = -inf_cells; dc <= inf_cells; ++dc) {
      if (dr == 0 && dc == 0) {
        continue;
      }
      int nr = row + dr;
      int nc = col + dc;
      if (nr < 0 || nr >= h || nc < 0 || nc >= w) {
        continue;
      }
      double dist = std::sqrt(static_cast<double>(dr * dr + dc * dc)) * res;
      if (dist > inflation_m) {
        continue;
      }
      int8_t cost = static_cast<int8_t>(std::max(1.0, 100.0 * std::pow(cost_decay, dist / res)));
      int idx = nr * w + nc;
      grid.data[idx] = std::max(grid.data[idx], cost);
    }
  }
}

}  // namespace costmap
