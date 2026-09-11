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

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <Eigen/Core>
#include <queue>
#include <vector>

#include "eidos/utils/voxel_pyramid.hpp"

namespace eidos::reloc
{

constexpr double kBnbPi = 3.14159265358979323846;  // M_PI isn't guaranteed under -std=c++17 -Wpedantic

// A scored 4-DOF pose hypothesis.
struct Hypothesis
{
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  int score = 0;  // Ternary [0, hit_weight*n] under Occupancy, cell-value sum
    // [0, 255*n] under DistanceField. See scorePoseAtLevel().
  double normalized = 0.0;  // score / max_possible, in [0, 1], valid in both modes.
  int hits = 0;  // Occupied-hit count (Occupancy) or non-zero-cell count (DistanceField).
  double hit_fraction = 0.0;  // hits / query point count.
};

// Tuning parameters for branchAndBound().
struct SearchConfig
{
  int hit_weight = 3;  // Occupied-hit weight under ScoreMode::Occupancy (raw score in [0, W*n]).
  ScoreMode score_mode = ScoreMode::DistanceField;
  // Prune children whose bound doesn't exceed best_score * prune_slack. Slack < 1.0 (vs.
  // textbook BnB pruning at best_score) so spatially distinct runner-ups survive for the
  // caller's NMS/uniqueness gate to compare against.
  double prune_slack = 0.8;
  double nms_radius = 5.0;  // Minimum separation (m) between distinct reported solutions.
  int max_solutions = 8;
  std::size_t max_nodes = 50000000;  // Safety cap on nodes expanded.
};

// Counters describing one branchAndBound() run.
struct SearchStats
{
  std::size_t nodes_expanded = 0;
  std::size_t nodes_pruned = 0;
  bool hit_node_cap = false;
  std::size_t greedy_evaluations = 0;  // Scoring calls made by the pre-loop greedy dive.
  std::size_t point_tests = 0;  // Query-point tests actually performed (post early-exit).
};

// One coarsest-level starting cell for the search frontier. Caller builds the root set (e.g. a
// trajectory corridor); this header only consumes it.
struct RootCell
{
  int64_t ix = 0;
  int64_t iy = 0;
  int64_t iz = 0;
};

// Shared yaw-discretisation math, used identically by branchAndBound() and both brute-force
// oracles so the three can't drift apart. Yaw step at a level is sized so a full bin's rotation
// displaces a point at max_range by about one voxel at that level's resolution; bin counts
// double per level going finer, matching how translation cells double, so the branching factor
// (8 translation children x 2 yaw children = 16) is exact.
struct YawDiscretization
{
  double max_range = 0.0;
  int coarsest_level = 0;
  int64_t coarse_bins = 1;

  // max_range == 0.0 signals a degenerate query (all points on the vertical axis) or an empty
  // pyramid -- callers must check and return no results rather than divide by zero.
  static YawDiscretization compute(const std::vector<Eigen::Vector3d> & query, const VoxelPyramid & pyramid)
  {
    YawDiscretization disc;
    double max_r = 0.0;
    for (const auto & q : query) {
      const double r = std::hypot(q.x(), q.y());
      if (r > max_r) max_r = r;
    }
    disc.max_range = max_r;
    if (pyramid.numLevels() <= 0 || max_r <= 0.0) return disc;

    disc.coarsest_level = pyramid.numLevels() - 1;
    const double resolution = pyramid.level(disc.coarsest_level).resolution;
    const double dtheta_coarsest = resolution / max_r;
    disc.coarse_bins = std::max<int64_t>(1, static_cast<int64_t>(std::ceil((2.0 * kBnbPi) / dtheta_coarsest)));
    return disc;
  }

  int64_t numBins(int level) const
  {
    return coarse_bins << (coarsest_level - level);
  }

  // Bin centres, not corners: (k + 0.5) * 2pi / n_l.
  double binCentre(int level, int64_t bin) const
  {
    const int64_t n = numBins(level);
    return (static_cast<double>(bin) + 0.5) * (2.0 * kBnbPi) / static_cast<double>(n);
  }
};

// Cell-centre translation for an integer voxel index. Node poses are represented at the cell
// centre, never the corner: that bounds a child's displacement from its parent to at most r_l
// per axis (half a cell of translation + half a bin of yaw), which is exactly the neighbourhood
// hitBound()'s 26-neighbourhood dilation covers. A corner-based representative would exceed that
// bound and let branch-and-bound prune away the true pose.
inline Eigen::Vector3d bnbCellCentre(int64_t ix, int64_t iy, int64_t iz, double resolution)
{
  return Eigen::Vector3d(
    (static_cast<double>(ix) + 0.5) * resolution,
    (static_cast<double>(iy) + 0.5) * resolution,
    (static_cast<double>(iz) + 0.5) * resolution);
}

// Scores a candidate pose against one pyramid level -- the single routine shared by the
// branch-and-bound bound computation, its leaf scoring, and both brute-force oracles, so they
// can't drift apart. Occupancy mode classifies each point occupied/free/unknown via
// hitBound()/isFreeBound() (level 0 uses the exact hit()/isFree()) and sums hit_weight/0/1;
// DistanceField mode sums each point's stored falloff cell value instead. Score is always
// non-negative, which is what keeps prune_slack * best_score a valid relaxed threshold.
//
// `min_required`: once the remaining unscored points can't possibly reach it, scoring stops
// early and returns a truncated partial count. Soundness invariant: any returned value >
// min_required - 1 was therefore always scored to completion (exact); only a truncated return is
// < min_required. Default 0 makes early exit impossible, which is what keeps the brute-force
// oracles (which don't pass this) exhaustive.
inline int scorePoseAtLevel(
  const VoxelPyramid & pyramid,
  const std::vector<Eigen::Vector3d> & query,
  const Eigen::Vector3d & translation,
  double yaw,
  int level,
  int hit_weight = 3,
  int min_required = 0,
  std::size_t * point_tests_out = nullptr,
  ScoreMode score_mode = ScoreMode::DistanceField)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const VoxelLevel & lvl = pyramid.level(level);
  const bool leaf = (level == 0);

  const int n = static_cast<int>(query.size());
  const int per_point_max = (score_mode == ScoreMode::DistanceField) ? 255 : hit_weight;

  int score = 0;

  const auto mapPoint = [&](int i) {
    const auto & q = query[static_cast<std::size_t>(i)];
    return Eigen::Vector3d(
      c * q.x() - s * q.y() + translation.x(), s * q.x() + c * q.y() + translation.y(), q.z() + translation.z());
  };

  if (score_mode == ScoreMode::DistanceField) {
    // A hash probe per point into a table far larger than cache (hundreds of MB at level 0 on a
    // km-scale map) is memory-latency bound, so this software-pipelines: prefetch kPrefetch
    // points ahead, score the current one against a line that's had time to arrive. Bit-identical
    // to the unpipelined form -- prefetch is advisory only.
    constexpr int kPrefetch = 8;  // power of two, so the ring index below is a mask
    int64_t pending[kPrefetch];
    const int primed = n < kPrefetch ? n : kPrefetch;
    for (int j = 0; j < primed; ++j) {
      pending[j] = lvl.scoreKey(mapPoint(j));
      lvl.scores.prefetch(pending[j]);
    }

    for (int i = 0; i < n; ++i) {
      const int slot = i & (kPrefetch - 1);
      const int64_t key = pending[slot];

      const int ahead = i + kPrefetch;
      if (ahead < n) {
        pending[slot] = lvl.scoreKey(mapPoint(ahead));
        lvl.scores.prefetch(pending[slot]);
      }

      score += lvl.scoreAtKey(key);
      if (point_tests_out != nullptr) ++(*point_tests_out);

      const int remaining = n - i - 1;
      if (score + remaining * per_point_max < min_required) return score;
    }
    return score;
  }

  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d p = mapPoint(i);
    if (leaf ? lvl.hit(p) : lvl.hitBound(p)) {
      score += hit_weight;
    } else if (!(leaf ? lvl.isFree(p) : lvl.isFreeBound(p))) {
      score += 1;  // unknown
    }
    if (point_tests_out != nullptr) ++(*point_tests_out);

    const int remaining = n - i - 1;
    if (score + remaining * per_point_max < min_required) return score;
  }
  return score;
}

// Per-point breakdown of a score, for diagnostics only.
struct ScoreBreakdown
{
  int hits = 0;  // Occupied points (Occupancy) or non-zero-cell points (DistanceField).
  int unknown = 0;  // Neither occupied nor free (Occupancy) or zero-cell (DistanceField).
  int free = 0;  // Known-free points. Always 0 under DistanceField.
  int raw = 0;  // Same value scorePoseAtLevel() would return for identical arguments.
  int max_possible = 0;  // hit_weight*n (Occupancy) or 255*n (DistanceField).
  double mean_cell_score = 0.0;  // raw / n.
};

// Exhaustive per-point breakdown (no min_required early exit), so callers can log a score's
// composition without perturbing search performance. Same classification as scorePoseAtLevel(),
// so breakdown.raw always matches what that function returns for identical arguments.
inline ScoreBreakdown scoreBreakdownAtLevel(
  const VoxelPyramid & pyramid,
  const std::vector<Eigen::Vector3d> & query,
  const Eigen::Vector3d & translation,
  double yaw,
  int level,
  int hit_weight = 3,
  ScoreMode score_mode = ScoreMode::DistanceField)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const VoxelLevel & lvl = pyramid.level(level);
  const bool leaf = (level == 0);
  const int n = static_cast<int>(query.size());

  ScoreBreakdown out;
  for (const auto & q : query) {
    const Eigen::Vector3d p(
      c * q.x() - s * q.y() + translation.x(), s * q.x() + c * q.y() + translation.y(), q.z() + translation.z());
    if (score_mode == ScoreMode::DistanceField) {
      const uint8_t cell = leaf ? lvl.scoreAt(p) : lvl.scoreBound(p);
      out.raw += cell;
      if (cell > 0) {
        ++out.hits;
      } else {
        ++out.unknown;
      }
    } else if (leaf ? lvl.hit(p) : lvl.hitBound(p)) {
      ++out.hits;
    } else if (leaf ? lvl.isFree(p) : lvl.isFreeBound(p)) {
      ++out.free;
    } else {
      ++out.unknown;
    }
  }
  if (score_mode == ScoreMode::DistanceField) {
    out.max_possible = 255 * n;
  } else {
    out.raw = hit_weight * out.hits + out.unknown;
    out.max_possible = hit_weight * n;
  }
  out.mean_cell_score = n > 0 ? static_cast<double>(out.raw) / static_cast<double>(n) : 0.0;
  return out;
}

// One frontier node. `bound` is scorePoseAtLevel()'s value at `level`: an upper bound for
// level > 0, the exact leaf score at level 0.
struct BnbNode
{
  int level = 0;
  int64_t ix = 0;
  int64_t iy = 0;
  int64_t iz = 0;
  int64_t yaw_bin = 0;
  int bound = 0;
};

// Max-heap by bound; ties broken by popping the DEEPER (smaller-level) node first, which pushes
// the search depth-first through tied regions and raises the incumbent sooner. Still exact
// best-first search -- only the tie-break order changes, never which nodes are admissible.
struct BnbNodeGreaterByBound
{
  bool operator()(const BnbNode & a, const BnbNode & b) const
  {
    if (a.bound != b.bound) return a.bound < b.bound;
    return a.level > b.level;
  }
};

// Best-first branch-and-bound search for the 4-DOF pose maximizing hits. Seeds the frontier with
// every root cell x every coarsest-level yaw bin, then repeatedly pops the highest-bound node:
// level-0 leaves are recorded as solutions, internal nodes expand into 16 children (8 translation
// x 2 yaw) whose bounds exceed the current prune threshold.
inline std::vector<Hypothesis> branchAndBound(
  const VoxelPyramid & pyramid,
  const std::vector<Eigen::Vector3d> & query,
  const std::vector<RootCell> & roots,
  const SearchConfig & cfg,
  SearchStats & stats)
{
  stats = SearchStats{};
  std::vector<Hypothesis> solutions;
  if (query.empty() || roots.empty() || pyramid.empty() || pyramid.numLevels() <= 0) return solutions;

  const YawDiscretization yaw_disc = YawDiscretization::compute(query, pyramid);
  if (yaw_disc.max_range <= 0.0) return solutions;

  const int coarsest = pyramid.numLevels() - 1;

  auto boundOf = [&](int level, int64_t ix, int64_t iy, int64_t iz, int64_t yaw_bin, int min_required) -> int {
    const double resolution = pyramid.level(level).resolution;
    const Eigen::Vector3d translation = bnbCellCentre(ix, iy, iz, resolution);
    const double yaw = yaw_disc.binCentre(level, yaw_bin);
    return scorePoseAtLevel(
      pyramid, query, translation, yaw, level, cfg.hit_weight, min_required, &stats.point_tests, cfg.score_mode);
  };

  auto leafHypothesis = [&](int64_t ix, int64_t iy, int64_t iz, int64_t yaw_bin) -> Hypothesis {
    const Eigen::Vector3d translation = bnbCellCentre(ix, iy, iz, pyramid.level(0).resolution);
    const double yaw = yaw_disc.binCentre(0, yaw_bin);
    const ScoreBreakdown breakdown =
      scoreBreakdownAtLevel(pyramid, query, translation, yaw, 0, cfg.hit_weight, cfg.score_mode);
    Hypothesis h;
    h.translation = translation;
    h.yaw = yaw;
    h.score = breakdown.raw;
    h.hits = breakdown.hits;
    h.hit_fraction = static_cast<double>(breakdown.hits) / static_cast<double>(query.size());
    h.normalized = breakdown.max_possible > 0
                     ? static_cast<double>(breakdown.raw) / static_cast<double>(breakdown.max_possible)
                     : 0.0;
    return h;
  };

  std::priority_queue<BnbNode, std::vector<BnbNode>, BnbNodeGreaterByBound> frontier;

  int best_score = 0;
  // -1, not 0: makes every non-negative bound admissible before any solution exists, and makes
  // prune_threshold + 1 == 0, under which scorePoseAtLevel()'s early exit can never fire.
  int prune_threshold = -1;

  const int64_t n_coarse = yaw_disc.numBins(coarsest);
  for (const auto & root : roots) {
    for (int64_t k = 0; k < n_coarse; ++k) {
      const int bound = boundOf(coarsest, root.ix, root.iy, root.iz, k, prune_threshold + 1);
      if (bound > prune_threshold) {
        frontier.push(BnbNode{coarsest, root.ix, root.iy, root.iz, k, bound});
      } else {
        ++stats.nodes_pruned;
      }
    }
  }

  // Greedy dive: establish a real incumbent before the main loop, so the min_required early exit
  // has something to bite on from the first pop instead of after tens of thousands of blind
  // expansions. Follows only the best-scoring child down to a leaf; exact (min_required=0
  // throughout), and doesn't count toward nodes_expanded/nodes_pruned.
  if (!frontier.empty()) {
    BnbNode cur = frontier.top();
    while (cur.level > 0) {
      const int child_level = cur.level - 1;
      BnbNode best_child;
      bool have_child = false;
      for (int dx = 0; dx <= 1; ++dx) {
        for (int dy = 0; dy <= 1; ++dy) {
          for (int dz = 0; dz <= 1; ++dz) {
            for (int dk = 0; dk <= 1; ++dk) {
              const int64_t cix = cur.ix * 2 + dx;
              const int64_t ciy = cur.iy * 2 + dy;
              const int64_t ciz = cur.iz * 2 + dz;
              const int64_t ck = cur.yaw_bin * 2 + dk;
              const int child_bound = boundOf(child_level, cix, ciy, ciz, ck, 0);
              ++stats.greedy_evaluations;
              if (!have_child || child_bound > best_child.bound) {
                best_child = BnbNode{child_level, cix, ciy, ciz, ck, child_bound};
                have_child = true;
              }
            }
          }
        }
      }
      cur = best_child;
    }

    const Hypothesis h = leafHypothesis(cur.ix, cur.iy, cur.iz, cur.yaw_bin);
    solutions.push_back(h);

    best_score = h.score;
    prune_threshold = static_cast<int>(std::floor(static_cast<double>(best_score) * cfg.prune_slack));
  }

  while (!frontier.empty()) {
    if (stats.nodes_expanded >= cfg.max_nodes) {
      stats.hit_node_cap = true;
      break;
    }

    const BnbNode node = frontier.top();
    frontier.pop();
    ++stats.nodes_expanded;

    // Frontier is ordered by bound descending: once the best remaining node can't beat the
    // threshold, nothing behind it can either.
    if (node.bound <= prune_threshold) break;

    if (node.level == 0) {
      const Hypothesis h = leafHypothesis(node.ix, node.iy, node.iz, node.yaw_bin);
      solutions.push_back(h);

      if (h.score > best_score) {
        best_score = h.score;
        prune_threshold = static_cast<int>(std::floor(static_cast<double>(best_score) * cfg.prune_slack));
      }
      continue;
    }

    const int child_level = node.level - 1;
    for (int dx = 0; dx <= 1; ++dx) {
      for (int dy = 0; dy <= 1; ++dy) {
        for (int dz = 0; dz <= 1; ++dz) {
          for (int dk = 0; dk <= 1; ++dk) {
            const int64_t cix = node.ix * 2 + dx;
            const int64_t ciy = node.iy * 2 + dy;
            const int64_t ciz = node.iz * 2 + dz;
            const int64_t ck = node.yaw_bin * 2 + dk;
            const int child_bound = boundOf(child_level, cix, ciy, ciz, ck, prune_threshold + 1);
            if (child_bound > prune_threshold) {
              frontier.push(BnbNode{child_level, cix, ciy, ciz, ck, child_bound});
            } else {
              ++stats.nodes_pruned;
            }
          }
        }
      }
    }
  }

  std::sort(
    solutions.begin(), solutions.end(), [](const Hypothesis & a, const Hypothesis & b) { return a.score > b.score; });

  std::vector<Hypothesis> accepted;
  for (const auto & candidate : solutions) {
    if (static_cast<int>(accepted.size()) >= cfg.max_solutions) break;
    bool far_enough = true;
    for (const auto & kept : accepted) {
      if ((candidate.translation - kept.translation).norm() <= cfg.nms_radius) {
        far_enough = false;
        break;
      }
    }
    if (far_enough) accepted.push_back(candidate);
  }
  return accepted;
}

// Exhaustively scores every root x every coarsest-level yaw bin (no pruning), via the same
// scorePoseAtLevel() call that seeds branchAndBound()'s frontier -- validates the scoring
// function independently of the bound-and-prune logic. `score_mode` must match what the pyramid
// was actually built with, or this reads an empty/wrong score channel.
inline std::vector<Hypothesis> bruteForceCoarse(
  const VoxelPyramid & pyramid,
  const std::vector<Eigen::Vector3d> & query,
  const std::vector<RootCell> & roots,
  int max_results,
  ScoreMode score_mode = ScoreMode::DistanceField)
{
  std::vector<Hypothesis> results;
  if (query.empty() || roots.empty() || pyramid.empty() || pyramid.numLevels() <= 0) return results;

  const YawDiscretization yaw_disc = YawDiscretization::compute(query, pyramid);
  if (yaw_disc.max_range <= 0.0) return results;

  const int coarsest = pyramid.numLevels() - 1;
  const double resolution = pyramid.level(coarsest).resolution;
  const int64_t n_coarse = yaw_disc.numBins(coarsest);
  constexpr int kHitWeight = 3;  // matches SearchConfig::hit_weight's default

  for (const auto & root : roots) {
    const Eigen::Vector3d translation = bnbCellCentre(root.ix, root.iy, root.iz, resolution);
    for (int64_t k = 0; k < n_coarse; ++k) {
      const double yaw = yaw_disc.binCentre(coarsest, k);
      const ScoreBreakdown breakdown =
        scoreBreakdownAtLevel(pyramid, query, translation, yaw, coarsest, kHitWeight, score_mode);
      Hypothesis h;
      h.translation = translation;
      h.yaw = yaw;
      h.score = breakdown.raw;
      h.hits = breakdown.hits;
      h.hit_fraction = static_cast<double>(breakdown.hits) / static_cast<double>(query.size());
      h.normalized =
        breakdown.max_possible > 0 ? static_cast<double>(h.score) / static_cast<double>(breakdown.max_possible) : 0.0;
      results.push_back(h);
    }
  }

  std::sort(
    results.begin(), results.end(), [](const Hypothesis & a, const Hypothesis & b) { return a.score > b.score; });
  if (max_results >= 0 && static_cast<int>(results.size()) > max_results) {
    results.resize(static_cast<std::size_t>(max_results));
  }
  return results;
}

// Exhaustively scores every level-0 cell in a box x every level-0 yaw bin. Reference oracle for
// branch-and-bound equivalence testing; intentionally slow (cubic in box size x query size x yaw
// bins) -- only suitable for small boxes in tests.
inline std::vector<Hypothesis> bruteForceLeaf(
  const VoxelPyramid & pyramid,
  const std::vector<Eigen::Vector3d> & query,
  const Eigen::Vector3d & box_min,
  const Eigen::Vector3d & box_max,
  int max_results,
  ScoreMode score_mode = ScoreMode::DistanceField)
{
  std::vector<Hypothesis> results;
  if (query.empty() || pyramid.empty() || pyramid.numLevels() <= 0) return results;

  const YawDiscretization yaw_disc = YawDiscretization::compute(query, pyramid);
  if (yaw_disc.max_range <= 0.0) return results;

  const VoxelLevel & leaf_level = pyramid.level(0);
  const double resolution = leaf_level.resolution;
  const double inv_resolution = leaf_level.inv_resolution;
  const int64_t ix_min = voxelIndex(box_min.x(), inv_resolution);
  const int64_t iy_min = voxelIndex(box_min.y(), inv_resolution);
  const int64_t iz_min = voxelIndex(box_min.z(), inv_resolution);
  const int64_t ix_max = voxelIndex(box_max.x(), inv_resolution);
  const int64_t iy_max = voxelIndex(box_max.y(), inv_resolution);
  const int64_t iz_max = voxelIndex(box_max.z(), inv_resolution);
  const int64_t n0 = yaw_disc.numBins(0);
  constexpr int kHitWeight = 3;

  for (int64_t ix = ix_min; ix <= ix_max; ++ix) {
    for (int64_t iy = iy_min; iy <= iy_max; ++iy) {
      for (int64_t iz = iz_min; iz <= iz_max; ++iz) {
        const Eigen::Vector3d translation = bnbCellCentre(ix, iy, iz, resolution);
        for (int64_t k = 0; k < n0; ++k) {
          const double yaw = yaw_disc.binCentre(0, k);
          const ScoreBreakdown breakdown =
            scoreBreakdownAtLevel(pyramid, query, translation, yaw, 0, kHitWeight, score_mode);
          Hypothesis h;
          h.translation = translation;
          h.yaw = yaw;
          h.score = breakdown.raw;
          h.hits = breakdown.hits;
          h.hit_fraction = static_cast<double>(breakdown.hits) / static_cast<double>(query.size());
          h.normalized = breakdown.max_possible > 0
                           ? static_cast<double>(h.score) / static_cast<double>(breakdown.max_possible)
                           : 0.0;
          results.push_back(h);
        }
      }
    }
  }

  std::sort(
    results.begin(), results.end(), [](const Hypothesis & a, const Hypothesis & b) { return a.score > b.score; });
  if (max_results >= 0 && static_cast<int>(results.size()) > max_results) {
    results.resize(static_cast<std::size_t>(max_results));
  }
  return results;
}

}  // namespace eidos::reloc
