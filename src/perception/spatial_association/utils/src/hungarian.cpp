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

#include "utils/hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace hungarian
{

std::vector<std::pair<size_t, size_t>> solve(
  const std::vector<double> & cost_matrix, size_t num_rows, size_t num_cols, double max_cost)
{
  if (num_rows == 0 || num_cols == 0) {
    return {};
  }

  // Pad to square matrix (n = max(rows, cols)).
  const size_t n = std::max(num_rows, num_cols);

  // Build padded cost matrix with forbidden entries set to a large value.
  const double kLarge = 1e18;
  std::vector<double> C(n * n, kLarge);
  for (size_t r = 0; r < num_rows; ++r) {
    for (size_t c = 0; c < num_cols; ++c) {
      C[r * n + c] = cost_matrix[r * num_cols + c];
    }
  }

  // Hungarian algorithm (Jonker-Volgenant shortest augmenting path variant).
  // u[i] and v[j] are dual variables (potentials) for rows and columns.
  // p[j] = row assigned to column j (-1 if unassigned).
  const int N = static_cast<int>(n);
  std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
  std::vector<int> p(n + 1, 0);   // p[j] = row assigned to col j (1-indexed; 0 = unassigned)
  std::vector<int> way(n + 1, 0); // way[j] = previous column in augmenting path

  for (int i = 1; i <= N; ++i) {
    // Start augmenting from row i.
    p[0] = i;
    int j0 = 0; // virtual column 0
    std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(n + 1, false);

    do {
      used[j0] = true;
      int i0 = p[j0];
      double delta = std::numeric_limits<double>::infinity();
      int j1 = -1;

      for (int j = 1; j <= N; ++j) {
        if (used[j]) continue;
        double cur = C[static_cast<size_t>(i0 - 1) * n + static_cast<size_t>(j - 1)] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }

      // Update potentials.
      for (int j = 0; j <= N; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }

      j0 = j1;
    } while (p[j0] != 0);

    // Trace back augmenting path.
    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  // Extract assignments within original matrix bounds and below max_cost.
  std::vector<std::pair<size_t, size_t>> result;
  result.reserve(std::min(num_rows, num_cols));
  for (size_t j = 1; j <= n; ++j) {
    size_t row = static_cast<size_t>(p[j]) - 1;
    size_t col = j - 1;
    if (row < num_rows && col < num_cols) {
      double c = cost_matrix[row * num_cols + col];
      if (c < max_cost) {
        result.emplace_back(row, col);
      }
    }
  }

  return result;
}

}  // namespace hungarian
