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

#ifndef ACCUMULATOR_HPP
#define ACCUMULATOR_HPP

#include <string>
#include <unordered_map>
#include <vector>

class Matches;  // Forward declaration

class Accumulator
{
public:
  Accumulator() = default;

  /**
    * @brief Updates accumulator with new matches.
    *
    * Accumulates all values (e.g. # gts, # tracks, total IoU, etc.)
    * in the accumulator with info from the new matches.
    *
    * @param m Matches from newest timestep.
    */
  void update(const Matches & m);

  /**
    * @brief Finds the id assignment that minimizes IDFP + IDFN.
    *
    * Solves a linear assignment problem on an IDFP + IDFN cost
    * matrix to find the minimum cost id assignment.
    *
    * @note The min cost assignment is used for IDF1 calculation.
    */
  void findCorrectMatches();

  /**
    * @brief Computes the IDFP + IDFN cost matrix.
    *
    * @return std::vector<std::vector<double>> The cost matrix.
    */
  std::vector<std::vector<double>> idfpfnMtx() const;

  /// @brief Creates string containing all member variable values.
  std::string toString() const;

  /// @brief MOTA metric calculation
  /// @return double MOTA = (FP + FN + IDSW) / (# gts)
  double MOTA() const;

  /// @brief MOTP metric calculation
  /// @return double MOTP = (total IoU) / (# matches)
  double MOTP() const;

  /// @brief IDF1 metric calculation
  /// @return double IDF1 = 2*IDTP / (2*IDTP + IDFP + IDFN)
  double IDF1() const;

  /// @brief IDP metric calculation
  /// @return double IDP = IDTP / (IDTP + IDFP)
  double IDP() const;

  /// @brief IDR metric calculation
  /// @return double IDR = IDTP / (IDTP + IDFN)
  double IDR() const;

  /// Getter functions to retrieve private member variable values
  double getTotDist() const;
  int getTotFP() const;
  int getTotFN() const;
  int getTotIDSW() const;
  int getTotIDTP() const;
  int getTotIDFP() const;
  int getTotIDFN() const;
  int getTotMatches() const;
  int getTotGts() const;
  int getTotTrks() const;
  const std::unordered_map<int, int> & getPrevMatches() const;
  const std::unordered_map<int, std::unordered_map<int, int>> & getMatchCounts() const;
  const std::unordered_map<int, int> & getGtOccs() const;
  const std::unordered_map<int, int> & getTrkOccs() const;

private:
  double tot_dist_{0.0};
  int tot_FP_{0};
  int tot_FN_{0};
  int tot_IDSW_{0};
  int tot_IDTP_{0};
  int tot_IDFP_{0};
  int tot_IDFN_{0};
  int tot_matches_{0};
  int tot_gts_{0};
  int tot_trks_{0};

  std::unordered_map<int, int> prev_matches_;
  std::unordered_map<int, std::unordered_map<int, int>> match_counts_;
  std::unordered_map<int, int> gt_occs_;
  std::unordered_map<int, int> trk_occs_;
};

#endif
