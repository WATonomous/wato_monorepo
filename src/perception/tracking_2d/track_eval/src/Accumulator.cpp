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

#include "track_eval/Accumulator.hpp"

#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "track_eval/lap.hpp"
#include "track_eval/Matches.hpp"

void Accumulator::update(const Matches & m)
{
  double dist = m.getDist();
  int num_matches = m.getNumMatches();
  int num_gts = m.getNumGts();
  int num_trks = m.getNumTrks();
  const std::unordered_map<int, int> & matched_ids = m.getMatchedIds();
  const std::vector<int> & unmatched_gts = m.getUnmatchedGts();
  const std::vector<int> & unmatched_trks = m.getUnmatchedTrks();

  tot_dist_ += dist;
  tot_matches_ += num_matches;
  tot_gts_ += num_gts;
  tot_trks_ += num_trks;
  tot_FP_ += num_trks - num_matches;
  tot_FN_ += num_gts - num_matches;

  for (const auto & [key, val] : prev_matches_) {
    auto new_match = matched_ids.find(key);
    if (new_match != matched_ids.end() && new_match->second != val && val >= 0) ++tot_IDSW_;
  }

  prev_matches_ = matched_ids;

  for (const auto & [key, val] : matched_ids) {
    ++gt_occs_[key];
    ++trk_occs_[val];
    ++match_counts_[key][val];
  }

  for (const auto & id : unmatched_gts) ++gt_occs_[id];
  for (const auto & id : unmatched_trks) ++trk_occs_[id];

  findCorrectMatches();
}

void Accumulator::findCorrectMatches()
{
  // Cost = IDFP + IDFN
  std::vector<std::vector<double>> cost = idfpfnMtx();
  std::vector<int> row_matches, col_matches;
  double opt = lapjv(cost, row_matches, col_matches);

  // IDTP = num_gts_ - IDFN = num_trks_ - IDFP
  tot_IDTP_ = (tot_gts_ + tot_trks_ - opt) / 2;
  tot_IDFP_ = tot_trks_ - tot_IDTP_;
  tot_IDFN_ = tot_gts_ - tot_IDTP_;
}

std::vector<std::vector<double>> Accumulator::idfpfnMtx() const
{
  // Matrix dimensions
  const int G = gt_occs_.size();
  const int T = trk_occs_.size();
  const int dim = G + T;

  // A valid solution should not assign any INVALID matchings
  const double INVALID = std::numeric_limits<double>::max();

  // (G + T) x (G + T) cost matrix; cost = IDFP + IDFN
  // [C X]
  // [Y Z]
  // where C (G x T) contains gt-trk costs
  //       X (G x G) contains gt occs on the diagonal and INVALID elsewhere
  //       Y (T x T) contains trk occs on the diagonal and INVALID elsewhere
  //       Z (T x G) the zero matrix
  std::vector<std::vector<double>> mtx(dim, std::vector<double>(dim, INVALID));

  // Vectors to avoid unordered_map lookups later
  std::vector<int> gt_ids, trk_ids;
  std::vector<int> gt_oc, trk_oc;
  gt_ids.reserve(G);
  trk_ids.reserve(T);
  gt_oc.reserve(G);
  trk_oc.reserve(T);

  // Populate vectors with appropriate ids and occurences
  for (const auto & p : gt_occs_) gt_ids.push_back(p.first);
  for (const auto & p : trk_occs_) trk_ids.push_back(p.first);
  for (const auto & p : gt_occs_) gt_oc.push_back(p.second);
  for (const auto & p : trk_occs_) trk_oc.push_back(p.second);

  // Update X and Y diagonals with occurences
  for (int g = 0; g < G; ++g) mtx[g][T + g] = gt_oc[g];
  for (int t = 0; t < T; ++t) mtx[G + t][t] = trk_oc[t];

  // Update C and Z blocks with costs and 0 respectively
  for (int g = 0; g < G; ++g) {
    int gt = gt_ids[g];
    int oc_gt = gt_oc[g];

    for (int t = 0; t < T; ++t) {
      int trk = trk_ids[t];
      int oc_trk = trk_oc[t];

      int match_count = 0;

      // If cannot find, then have not matched before
      if (auto it1 = match_counts_.find(gt); it1 != match_counts_.end()) {
        if (auto it2 = it1->second.find(trk); it2 != it1->second.end()) {
          match_count = it2->second;
        }
      }

      // cost_ij = IDFP_ij + IDFN_ij
      //         = (occg_i - matches_ij) + (occt_j - matches_ij)
      mtx[g][t] = oc_gt + oc_trk - 2 * match_count;
      mtx[G + t][T + g] = 0.0;  // Fill Z with 0.0
    }
  }
  return mtx;
}

std::string Accumulator::toString() const
{
  std::string s = "Total dist: " + std::to_string(tot_dist_) + "\n" + "FP: " + std::to_string(tot_FP_) + "\n" +
                  "FN: " + std::to_string(tot_FN_) + "\n" + "IDSW: " + std::to_string(tot_IDSW_) + "\n" +
                  "IDTP: " + std::to_string(tot_IDTP_) + "\n" + "IDFP: " + std::to_string(tot_IDFP_) + "\n" +
                  "IDFN: " + std::to_string(tot_IDFN_) + "\n" + "# matches: " + std::to_string(tot_matches_) + "\n" +
                  "# gts: " + std::to_string(tot_gts_) + "\n" + "# trks: " + std::to_string(tot_trks_) + "\n" +
                  "Matched ids:\n";
  for (const auto & [key, val] : prev_matches_) s += std::to_string(key) + ": " + std::to_string(val) + "\n";
  s += "\nGT occs:\n";
  for (const auto & [key, val] : gt_occs_) s += std::to_string(key) + ": " + std::to_string(val) + "\n";
  s += "\nTrk occs:\n";
  for (const auto & [key, val] : trk_occs_) s += std::to_string(key) + ": " + std::to_string(val) + "\n";
  s += "\n";
  return s;
}

double Accumulator::MOTA() const
{
  if (tot_gts_ <= 0) return 1.0;
  return 1.0 - static_cast<double>(tot_FP_ + tot_FN_ + tot_IDSW_) / tot_gts_;
}

double Accumulator::MOTP() const
{
  if (tot_matches_ <= 0) return 1.0;
  return tot_dist_ / tot_matches_;
}

double Accumulator::IDF1() const
{
  // 2*IDTP + IDFP + IDFN = (IDTP + IDFP) + (IDTP + IDFN)
  //                      = # gts + # tracks
  int denom = tot_gts_ + tot_trks_;
  if (denom <= 0) return 1.0;
  return 2.0 * tot_IDTP_ / denom;
}

double Accumulator::IDP() const
{
  if (tot_trks_ <= 0) return 1.0;
  return static_cast<double>(tot_IDTP_) / tot_trks_;
}

double Accumulator::IDR() const
{
  if (tot_gts_ <= 0) return 1.0;
  return static_cast<double>(tot_IDTP_) / tot_gts_;
}

// Getter functions
double Accumulator::getTotDist() const
{
  return tot_dist_;
}

int Accumulator::getTotFP() const
{
  return tot_FP_;
}

int Accumulator::getTotFN() const
{
  return tot_FN_;
}

int Accumulator::getTotIDSW() const
{
  return tot_IDSW_;
}

int Accumulator::getTotIDTP() const
{
  return tot_IDTP_;
}

int Accumulator::getTotIDFP() const
{
  return tot_IDFP_;
}

int Accumulator::getTotIDFN() const
{
  return tot_IDFN_;
}

int Accumulator::getTotMatches() const
{
  return tot_matches_;
}

int Accumulator::getTotGts() const
{
  return tot_gts_;
}

int Accumulator::getTotTrks() const
{
  return tot_trks_;
}

const std::unordered_map<int, int> & Accumulator::getPrevMatches() const
{
  return prev_matches_;
}

const std::unordered_map<int, std::unordered_map<int, int>> & Accumulator::getMatchCounts() const
{
  return match_counts_;
}

const std::unordered_map<int, int> & Accumulator::getGtOccs() const
{
  return gt_occs_;
}

const std::unordered_map<int, int> & Accumulator::getTrkOccs() const
{
  return trk_occs_;
}
