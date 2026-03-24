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
#include <limits>
#include <vector>

namespace hungarian
{

std::vector<int> solve(const std::vector<double> & cost, size_t dim)
{
  if (dim == 0) return {};
  if (cost.size() != dim * dim) return std::vector<int>(dim, -1);

  // Dense O(n^3) primal-dual Hungarian (1-indexed internals).
  std::vector<double> u(dim + 1, 0.0), v(dim + 1, 0.0);
  std::vector<size_t> p(dim + 1, 0), way(dim + 1, 0);

  for (size_t i = 1; i <= dim; ++i) {
    p[0] = i;
    size_t j0 = 0;
    std::vector<double> minv(dim + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(dim + 1, false);
    do {
      used[j0] = true;
      const size_t i0 = p[j0];
      double delta = std::numeric_limits<double>::infinity();
      size_t j1 = 0;
      for (size_t j = 1; j <= dim; ++j) {
        if (used[j]) continue;
        const double cur = cost[(i0 - 1) * dim + (j - 1)] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (size_t j = 0; j <= dim; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const size_t j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assignment(dim, -1);
  for (size_t j = 1; j <= dim; ++j) {
    if (p[j] == 0) continue;
    assignment[p[j] - 1] = static_cast<int>(j - 1);
  }
  return assignment;
}

}  // namespace hungarian
