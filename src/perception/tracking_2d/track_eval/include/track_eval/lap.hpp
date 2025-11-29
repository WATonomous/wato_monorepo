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

#ifndef LAP_HPP
#define LAP_HPP

#include <limits>
#include <vector>

/**
 * @brief Wrapper for byte_track::lapjv_internal.
 *
 * Prepares a cost matrix before passing it to
 * byte_track::lapjv_internal, which requires a square cost matrix.
 * 1) If the matrix is rectangular, extend to a square marix.
 * 2) If a cost_limit is set, extend to include it.
 *
 * @param cost The cost matrix.
 * @param rowsol Stores the matches per row of the solution.
 * @param colsol Stores the matches per column of the solution.
 * @param cost_limit Maximum cost of a match.
 * @return double The total cost of the optimal solution.
 *
 * @note Modified version of execLapjv from Vertical-Beach/ByteTrack-cpp (MIT License).
 *       see BYTETRACK_LICENSE file for more details.
 */
double lapjv(
  std::vector<std::vector<double>> cost,
  std::vector<int> & rowsol,
  std::vector<int> & colsol,
  double cost_limit = std::numeric_limits<double>::max());

#endif
