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
#include <vector>

namespace hungarian
{

/**
 * @brief Solve linear assignment on a square cost matrix.
 *
 * @param cost Row-major square matrix of size @p dim * @p dim.
 * @param dim Matrix dimension.
 * @return Assignment vector where result[row] = assigned column, or -1 when unassigned.
 */
std::vector<int> solve(const std::vector<double> & cost, size_t dim);

}  // namespace hungarian

#endif
