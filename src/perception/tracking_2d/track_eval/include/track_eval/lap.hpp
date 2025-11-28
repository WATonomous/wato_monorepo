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
 * 2) If a match_threshold is set, extend to include it.
 *
 * @param cost The cost matrix.
 * @param row_matches Stores the matches per row of the solution.
 * @param col_matches Stores the matches per column of the solution.
 * @param match_threshold Maximum cost of a match (default = 0.8).
 * @return double The total cost of the optimal solution.
 *
 * @note Modified version of execLapjv from Vertical-Beach/ByteTrack-cpp (MIT License).
 *       see BYTETRACK_LICENSE file for more details.
 */
double lapjv(
  std::vector<std::vector<double>> cost,
  std::vector<int> & row_matches,
  std::vector<int> & col_matches,
  double match_threshold = std::numeric_limits<double>::max());

#endif
