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

#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// Modified version of execLapjv from Vertical-Beach/ByteTrack-cpp (MIT License)
// See BYTETRACK_LICENSE file for more details
double lapjv(
  std::vector<std::vector<double>> cost,
  std::vector<int> & row_matches,
  std::vector<int> & col_matches,
  double match_threshold)
{
  int num_rows = cost.size();
  int num_cols = cost[0].size();
  row_matches.resize(num_rows);
  col_matches.resize(num_cols);

  int n = 0;
  if (num_rows == num_cols) n = num_rows;

  // If rectangular or active match threshold
  if (num_rows != num_cols || match_threshold < std::numeric_limits<double>::max()) {
    n = num_rows + num_cols;

    // Expand to larger square matrix
    cost.resize(n);
    for (int i = 0; i < n; ++i) cost[i].resize(n);

    for (int i = num_rows; i < n; ++i) {
      for (int j = 0; j < num_cols; ++j) cost[i][j] = match_threshold / 2;
    }

    for (int i = 0; i < num_rows; ++i) {
      for (int j = num_cols; j < n; ++j) cost[i][j] = match_threshold / 2;
    }

    for (int i = num_rows; i < n; ++i) {
      for (int j = num_cols; j < n; ++j) cost[i][j] = 0.0;
    }
  }

  std::vector<double *> cost_mtx(n);
  for (int i = 0; i < n; ++i) cost_mtx[i] = cost[i].data();

  std::vector<int> r_match(n);
  std::vector<int> c_match(n);

  int ret = byte_track::lapjv_internal(n, cost_mtx.data(), r_match.data(), c_match.data());
  if (ret != 0) {
    RCLCPP_WARN(rclcpp::get_logger("LAPJV"), "Invalid lapjv solution, returned dummy solution");
    return -1;
  }

  double min_cost = 0.0;

  if (n != num_rows) {
    for (int i = 0; i < n; ++i) {
      if (r_match[i] >= num_cols) r_match[i] = -1;
      if (c_match[i] >= num_rows) c_match[i] = -1;
    }
  }

  for (int i = 0; i < num_rows; ++i) row_matches[i] = r_match[i];
  for (int i = 0; i < num_cols; ++i) col_matches[i] = c_match[i];

  for (int i = 0; i < num_rows; ++i) {
    if (row_matches[i] != -1) min_cost += cost[i][row_matches[i]];
  }

  return min_cost;
}
