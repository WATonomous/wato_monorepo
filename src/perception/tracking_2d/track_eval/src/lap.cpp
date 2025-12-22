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

#include "track_eval/lap.hpp"

#include <ByteTrack/lapjv.h>

#include <algorithm>
#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// Modified version of execLapjv from Vertical-Beach/ByteTrack-cpp (MIT License)
// See BYTETRACK_LICENSE file for more details
double lapjv(
  std::vector<std::vector<double>> cost, std::vector<int> & rowsol, std::vector<int> & colsol, double cost_limit)
{
  // Prepare vectors to store row and column matches
  int n_rows = cost.size();
  int n_cols = cost[0].size();
  rowsol.resize(n_rows);
  colsol.resize(n_cols);

  int n = 0;
  if (n_rows == n_cols) n = n_rows;

  // If rectangular or active match threshold
  if (n_rows != n_cols || cost_limit < std::numeric_limits<double>::max()) {
    n = n_rows + n_cols;

    double invalid = 0.0;

    if (cost_limit < std::numeric_limits<double>::max()) {
      invalid = cost_limit / 2;
    } else {
      for (const auto & row : cost) {
        for (double val : row) invalid = std::max(invalid, val);
      }
      ++invalid;
    }

    // Expand to larger square matrix
    cost.resize(n);
    for (int i = 0; i < n; ++i) cost[i].resize(n);

    // Fill new quadrants
    // Top-right and bottom-left set to invalid
    for (int i = n_rows; i < n; ++i) {
      for (int j = 0; j < n_cols; ++j) cost[i][j] = invalid;
    }

    for (int i = 0; i < n_rows; ++i) {
      for (int j = n_cols; j < n; ++j) cost[i][j] = invalid;
    }

    // Bottom-right set to all 0
    for (int i = n_rows; i < n; ++i) {
      for (int j = n_cols; j < n; ++j) cost[i][j] = 0.0;
    }
  }

  // Define ptrs for lapjv_internal
  std::vector<double *> cost_ptr(n);
  for (int i = 0; i < n; ++i) cost_ptr[i] = cost[i].data();

  std::vector<int> x_c(n);
  std::vector<int> y_c(n);

  int ret = byte_track::lapjv_internal(n, cost_ptr.data(), x_c.data(), y_c.data());
  if (ret != 0) {
    RCLCPP_WARN(rclcpp::get_logger("LAPJV"), "Invalid lapjv solution, returning dummy solution");
    return -1;
  }

  // Calculate total cost and copy matched indices into rowsol and colsol
  double opt = 0.0;

  if (n != n_rows) {
    for (int i = 0; i < n; ++i) {
      if (x_c[i] >= n_cols) x_c[i] = -1;
      if (y_c[i] >= n_rows) y_c[i] = -1;
    }
  }

  for (int i = 0; i < n_rows; ++i) rowsol[i] = x_c[i];
  for (int i = 0; i < n_cols; ++i) colsol[i] = y_c[i];

  for (int i = 0; i < n_rows; ++i) {
    if (rowsol[i] != -1) opt += cost[i][rowsol[i]];
  }

  return opt;
}
