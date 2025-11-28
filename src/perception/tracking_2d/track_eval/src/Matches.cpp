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

#include "track_eval/Matches.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

int Matches::getID(int idx, const std::vector<vision_msgs::msg::Detection2D> & dets) const
{
  int num_dets = dets.size();
  if (idx < 0 || idx >= num_dets) return -1;
  return std::stoi(dets[idx].id);
}

Matches::Matches(
  double cost,
  const std::vector<int> & row_matches,
  const std::vector<int> & col_matches,
  const std::vector<vision_msgs::msg::Detection2D> & gts,
  const std::vector<vision_msgs::msg::Detection2D> & trks)
: num_gts_(row_matches.size())
, num_trks_(col_matches.size())
{
  int num_rows = row_matches.size(), num_cols = col_matches.size();
  num_matches_ = 0;
  for (int i = 0; i < num_rows; ++i) {
    int trk_id = getID(row_matches[i], trks);
    if (trk_id >= 0) {
      ++num_matches_;
      matched_ids_.insert(std::make_pair(getID(i, gts), trk_id));
    } else {
      unmatched_gts_.push_back(getID(i, gts));
    }
  }
  for (int j = 0; j < num_cols; ++j) {
    int row_id = getID(col_matches[j], gts);
    if (row_id < 0) {
      unmatched_trks_.push_back(getID(j, trks));
    }
  }
  dist_ = num_matches_ - cost;
}

std::string Matches::toString() const
{
  std::string s = "# matches: " + std::to_string(num_matches_) + "\n" + "# gts: " + std::to_string(num_gts_) + "\n" +
                  "# trks: " + std::to_string(num_trks_) + "\n" + "Dist: " + std::to_string(dist_) + "\n" +
                  "Matched ids:\n";
  for (const auto & [key, val] : matched_ids_) s += std::to_string(key) + ": " + std::to_string(val) + "\n";
  s += "\nUnmatched gts:\n";
  for (const auto & id : unmatched_gts_) s += std::to_string(id) + "\n";
  s += "\nUnmatched trks:\n";
  for (const auto & id : unmatched_trks_) s += std::to_string(id) + "\n";
  s += "\n";
  return s;
}

double Matches::getDist() const
{
  return dist_;
}

int Matches::getNumMatches() const
{
  return num_matches_;
}

int Matches::getNumGts() const
{
  return num_gts_;
}

int Matches::getNumTrks() const
{
  return num_trks_;
}

const std::unordered_map<int, int> & Matches::getMatchedIds() const
{
  return matched_ids_;
}

const std::vector<int> & Matches::getUnmatchedGts() const
{
  return unmatched_gts_;
}

const std::vector<int> & Matches::getUnmatchedTrks() const
{
  return unmatched_trks_;
}
