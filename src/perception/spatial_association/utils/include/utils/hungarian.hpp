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

#ifndef HUNGARIAN_HPP
#define HUNGARIAN_HPP

#include <cstddef>
#include <utility>
#include <vector>

namespace wato::perception::hungarian
{

/**
 * @brief Solve the linear assignment problem (minimum cost) using the Hungarian/Munkres algorithm.
 *
 * Given an NxM cost matrix, finds the assignment of rows to columns that minimizes total cost.
 * Entries with cost >= @p max_cost are treated as forbidden (will not be assigned).
 *
 * @param cost_matrix Row-major NxM cost matrix (N rows, M columns).
 * @param num_rows Number of rows (candidates).
 * @param num_cols Number of columns (detections).
 * @param max_cost Assignments with cost >= this value are excluded from the result.
 * @return Vector of (row, col) assignments with cost < max_cost.
 */
std::vector<std::pair<size_t, size_t>> solve(
  const std::vector<double> & cost_matrix, size_t num_rows, size_t num_cols, double max_cost = 1e9);

}  // namespace wato::perception::hungarian

#endif
