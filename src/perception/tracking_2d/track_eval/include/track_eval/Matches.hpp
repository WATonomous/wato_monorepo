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

#ifndef MATCHES_HPP
#define MATCHES_HPP

#include <string>
#include <unordered_map>
#include <vector>

#include <vision_msgs/msg/detection2_d_array.hpp>

class Matches
{
public:
  Matches() = default;

  /**
     * @brief Construct a new Matches object from LAP solution.
     *
     * @param cost Total cost of the optimal linear assignment.
     * @param row_matches, col_matches The row and column match solutions.
     * @param gts, trks Original Detection2D messages for id matching.
     */
  Matches(
    double cost,
    const std::vector<int> & row_matches,
    const std::vector<int> & col_matches,
    const std::vector<vision_msgs::msg::Detection2D> & gts,
    const std::vector<vision_msgs::msg::Detection2D> & trks);

  /// @brief Gets string with all Matches info
  std::string toString() const;

  /// Getter functions to retrieve private values
  double getDist() const;
  int getNumMatches() const;
  int getNumGts() const;
  int getNumTrks() const;
  const std::unordered_map<int, int> & getMatchedIds() const;
  const std::vector<int> & getUnmatchedGts() const;
  const std::vector<int> & getUnmatchedTrks() const;

private:
  /// @brief Get ground truth or track id by index.
  int getID(int idx, const std::vector<vision_msgs::msg::Detection2D> & dets) const;

  double dist_{0.0};
  int num_matches_{0};
  int num_gts_{0};
  int num_trks_{0};
  std::unordered_map<int, int> matched_ids_;
  std::vector<int> unmatched_gts_;
  std::vector<int> unmatched_trks_;
};

#endif
