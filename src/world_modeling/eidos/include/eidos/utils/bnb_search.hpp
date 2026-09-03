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
#include <queue>
#include <vector>

#include <Eigen/Core>

#include "eidos/utils/voxel_pyramid.hpp"

namespace eidos::reloc
{

/// @brief Pi, spelled out locally so this header does not depend on the
/// non-standard `M_PI` macro (which is not guaranteed by <cmath> under
/// `-std=c++17 -Wpedantic`).
constexpr double kBnbPi = 3.14159265358979323846;

/// @brief A scored 4-DOF pose hypothesis.
struct Hypothesis
{
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();  ///< Map-frame translation (m).
  double yaw = 0.0;  ///< Map-frame yaw about Z (rad).
  int score = 0;  ///< Raw score: ternary in `[0, hit_weight * n]` under Occupancy, or the
                  ///< sum of per-point distance-field cell values in `[0, 255 * n]` under
                  ///< DistanceField; see scorePoseAtLevel().
  double normalized = 0.0;  ///< `score / max_possible`, in `[0, 1]`, valid in both modes
                  ///< (see ScoreBreakdown::max_possible).
  int hits = 0;  ///< Occupancy: occupied-hit count. DistanceField: count of points whose
                  ///< cell value is non-zero, i.e. within `df_truncation_voxels`.
  double hit_fraction = 0.0;  ///< `hits / query point count`, for diagnostics.
};

/// @brief Tuning parameters for `branchAndBound()`.
struct SearchConfig
{
  /// Per-point weight W for an occupied hit in the ternary score (raw score in
  /// `[0, W*n]`): occupied contributes W, known-free contributes 0, unknown
  /// contributes 1. See scorePoseAtLevel() for the full soundness argument. Only
  /// consulted under `ScoreMode::Occupancy`.
  int hit_weight = 3;
  /// Scoring model this search evaluates against -- see ScoreMode. DistanceField (the
  /// new default) sums per-point cell values in `[0, 255]`; Occupancy reproduces the
  /// original ternary score bit-for-bit. Threaded through to every scorePoseAtLevel() /
  /// scoreBreakdownAtLevel() call this search makes, so the bound computation, the leaf
  /// scoring, and the pyramid this search runs against must agree -- callers whose
  /// pyramid fell back to Occupancy after a distance-field budget overflow should read
  /// `VoxelPyramid::effectiveScoreMode()` rather than assume this default.
  ScoreMode score_mode = ScoreMode::DistanceField;
  /// Prune children whose bound does not exceed `best_score * prune_slack`.
  /// Textbook branch-and-bound prunes at `best_score` itself; the slack here
  /// is intentional so that spatially distinct runner-up hypotheses survive
  /// long enough to be collected. Those runners-up are exactly what the
  /// caller's non-maximum-suppression / uniqueness gate needs to compare the
  /// best solution against -- pruning at `best_score` would leave that gate
  /// with nothing else to look at.
  double prune_slack = 0.8;
  /// Minimum Euclidean separation (m) between distinct reported solutions.
  double nms_radius = 5.0;
  /// Maximum number of distinct hypotheses to return.
  int max_solutions = 8;
  /// Safety cap on the number of nodes expanded before aborting the search.
  std::size_t max_nodes = 50000000;
};

/// @brief Counters describing one `branchAndBound()` run.
struct SearchStats
{
  std::size_t nodes_expanded = 0;  ///< Nodes popped from the frontier and processed.
  std::size_t nodes_pruned = 0;  ///< Candidate children rejected without being queued.
  bool hit_node_cap = false;  ///< True if `SearchConfig::max_nodes` cut the search short.
  std::size_t greedy_evaluations = 0;  ///< Scoring calls made by the pre-loop greedy dive.
  std::size_t point_tests = 0;  ///< Total query-point tests actually performed (post early-exit).
};

/// @brief One coarsest-level starting cell for the search frontier.
///
/// The caller is responsible for building the set of roots (e.g. a
/// trajectory corridor at the coarsest pyramid resolution); this header only
/// consumes them.
struct RootCell
{
  int64_t ix = 0;  ///< Coarsest-level voxel index along x.
  int64_t iy = 0;  ///< Coarsest-level voxel index along y.
  int64_t iz = 0;  ///< Coarsest-level voxel index along z.
};

/**
 * @brief Shared yaw-discretisation math, used identically by the branch-and-bound
 * search and both brute-force oracles so the three cannot drift apart.
 *
 * The yaw step at a level is sized so that a full bin's worth of rotation
 * displaces a point at `max_range` by about one voxel at that level's
 * resolution: `dtheta_l = r_l / max_range`. Bin counts double per level going
 * from coarse to fine, mirroring how translation cells double per level, so
 * that the 16-way branching factor (8 translation children x 2 yaw children)
 * is exact.
 */
struct YawDiscretization
{
  double max_range = 0.0;  ///< Max horizontal norm over the query points (m).
  int coarsest_level = 0;  ///< Index of the coarsest pyramid level (`numLevels() - 1`).
  int64_t coarse_bins = 1;  ///< Number of yaw bins at `coarsest_level`.

  /**
   * @brief Compute the shared discretisation for a query set against a pyramid.
   *
   * @param query Body-frame, gravity-de-tilted query points.
   * @param pyramid Pyramid supplying per-level resolutions.
   * @return Populated discretisation. `max_range == 0.0` signals a degenerate
   *   query set (all points on the vertical axis) or an empty pyramid; callers
   *   must check for this and return no results rather than divide by zero.
   */
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
    disc.coarse_bins =
      std::max<int64_t>(1, static_cast<int64_t>(std::ceil((2.0 * kBnbPi) / dtheta_coarsest)));
    return disc;
  }

  /**
   * @brief Number of yaw bins at a level.
   *
   * Bin counts double per level going down: `n_l = coarse_bins * 2^(coarsest_level - l)`.
   *
   * @param level Pyramid level, 0 is finest.
   * @return Bin count at that level.
   */
  int64_t numBins(int level) const
  {
    return coarse_bins << (coarsest_level - level);
  }

  /**
   * @brief Bin-centre yaw angle for a bin index at a level.
   *
   * Bin centres, not corners, are used: `(k + 0.5) * 2*pi / n_l`.
   *
   * @param level Pyramid level, 0 is finest.
   * @param bin Bin index in `[0, numBins(level))`.
   * @return Yaw angle in radians.
   */
  double binCentre(int level, int64_t bin) const
  {
    const int64_t n = numBins(level);
    return (static_cast<double>(bin) + 0.5) * (2.0 * kBnbPi) / static_cast<double>(n);
  }
};

/**
 * @brief Cell-centre translation for an integer voxel index at a resolution.
 *
 * Node poses are represented at the CELL CENTRE, never the cell corner. This
 * is what lets `VoxelLevel::hitBound()`'s 26-neighbourhood dilation serve as a
 * valid upper bound: a child's representative pose differs from its parent's
 * by at most `r_l/2` per translation axis (half a cell) plus at most
 * `max_range * dtheta_l / 2 = r_l/2` from the half-bin yaw offset, for a total
 * per-axis displacement bounded by `r_l`. That means a query point's voxel
 * index at resolution `r_l` can change by at most one step between a node and
 * any of its descendants -- exactly the neighbourhood the dilation covers. A
 * corner-based representative would allow up to a full `r_l` of translation
 * displacement on top of the yaw offset and would break the bound, letting
 * branch-and-bound prune away the true pose.
 *
 * @param ix Voxel index along x.
 * @param iy Voxel index along y.
 * @param iz Voxel index along z.
 * @param resolution Voxel edge length at this level (m).
 * @return Map-frame translation of the cell centre.
 */
inline Eigen::Vector3d bnbCellCentre(int64_t ix, int64_t iy, int64_t iz, double resolution)
{
  return Eigen::Vector3d(
    (static_cast<double>(ix) + 0.5) * resolution,
    (static_cast<double>(iy) + 0.5) * resolution,
    (static_cast<double>(iz) + 0.5) * resolution);
}

/**
 * @brief Score a candidate pose against one pyramid level.
 *
 * This is the single scoring routine shared by the branch-and-bound bound
 * computation, its leaf scoring, and both brute-force oracles, so they cannot
 * drift apart. Query points are transformed by a yaw rotation about Z
 * followed by the translation, then classified per point into one of three
 * states -- occupied, known-free, or unknown -- and summed into a ternary
 * score (see the module-level contract this header implements): occupied
 * contributes `hit_weight`, known-free contributes 0, unknown contributes 1.
 * Occupied/free are tested via `hitBound()`/`isFreeBound()` (upper/lower
 * bounds) for any level above 0, and the exact `hit()`/`isFree()` at level 0.
 *
 * Because a query point can never contribute more than `hit_weight`, the
 * score is always in `[0, hit_weight * n]` -- deliberately non-negative, so
 * that `prune_slack * best_score` stays a valid relaxed prune threshold.
 *
 * BACKWARD COMPATIBILITY: when the pyramid was built with
 * `build_free_space = false`, no voxel is ever known-free, so every point is
 * either occupied or unknown and the score reduces to
 * `hit_weight * hits + (n - hits) = (hit_weight - 1) * hits + n`, which is
 * strictly monotone in the hit count for any `hit_weight > 1`. Ranking by this
 * score is therefore bit-identical to ranking by the old pure hit count.
 *
 * @param pyramid Pyramid to score against.
 * @param query Body-frame, gravity-de-tilted query points.
 * @param translation Candidate map-frame translation.
 * @param yaw Candidate map-frame yaw about Z (rad).
 * @param level Pyramid level to score against, 0 is finest.
 * @param hit_weight Per-point weight W for an occupied hit. Defaults to 3, matching
 *   `SearchConfig::hit_weight`, so callers that only care about the binary/ternary
 *   ranking do not need to thread a config through. Only consulted under
 *   `ScoreMode::Occupancy`; under `ScoreMode::DistanceField` a point's contribution is
 *   the stored cell value in `[0, 255]` instead, via `scoreAt()`/`scoreBound()`.
 * @param min_required Optional score floor. Once the remaining, unscored points
 *   could not possibly bring the running score up to `min_required`, scoring
 *   stops early and the (necessarily truncated) partial count is returned. The
 *   maximum a single remaining point can contribute is `hit_weight` under
 *   `ScoreMode::Occupancy` or `255` under `ScoreMode::DistanceField`, so the
 *   impossibility test is `score + remaining * per_point_max < min_required`.
 *   Defaults to 0, under which that bound can never hold, so scoring always
 *   runs to completion -- this is what keeps the brute-force oracles, which
 *   call this function without the argument, exhaustive.
 *
 *   SOUNDNESS INVARIANT: early exit only fires when reaching `min_required` is
 *   arithmetically impossible, so any value `> min_required - 1` (equivalently
 *   any value a caller compares against a threshold as `> min_required - 1`)
 *   returned here was necessarily scored to completion and is EXACT. A
 *   truncated return is always `< min_required`. Callers that reject nodes
 *   scoring `< min_required` therefore never mistake a truncated partial count
 *   for a true bound, and never push a node whose score was truncated.
 * @param point_tests_out Optional accumulator incremented once per query point
 *   actually tested, so callers can measure the early-exit benefit.
 * @param score_mode Scoring model to evaluate. `Occupancy` reproduces the original
 *   ternary score bit-for-bit (see the BACKWARD COMPATIBILITY note above) and is
 *   UNCHANGED by this parameter's addition. `DistanceField` (the default, matching
 *   `SearchConfig::score_mode`) instead sums each point's stored cell value via
 *   `scoreAt()`/`scoreBound()`, ignoring the free-space channel entirely -- a point far
 *   from all structure already scores near 0 continuously, which is what the ternary
 *   unknown/free split was approximating.
 * @return The score (ternary under Occupancy, cell-value sum under DistanceField), or a
 *   truncated partial count strictly below `min_required` if scoring was abandoned early.
 */
inline int scorePoseAtLevel(
  const VoxelPyramid & pyramid, const std::vector<Eigen::Vector3d> & query,
  const Eigen::Vector3d & translation, double yaw, int level, int hit_weight = 3, int min_required = 0,
  std::size_t * point_tests_out = nullptr, ScoreMode score_mode = ScoreMode::DistanceField)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const VoxelLevel & lvl = pyramid.level(level);
  const bool leaf = (level == 0);

  const int n = static_cast<int>(query.size());
  // The maximum a single remaining point could still contribute, for the min_required
  // impossibility test below: hit_weight under the ternary Occupancy score, 255 (a
  // uint8_t cell value) under DistanceField.
  const int per_point_max = (score_mode == ScoreMode::DistanceField) ? 255 : hit_weight;

  int score = 0;
  for (int i = 0; i < n; ++i) {
    const auto & q = query[static_cast<std::size_t>(i)];
    const Eigen::Vector3d p(
      c * q.x() - s * q.y() + translation.x(), s * q.x() + c * q.y() + translation.y(), q.z() + translation.z());
    if (score_mode == ScoreMode::DistanceField) {
      score += leaf ? lvl.scoreAt(p) : lvl.scoreBound(p);
    } else if (leaf ? lvl.hit(p) : lvl.hitBound(p)) {
      score += hit_weight;
    } else if (!(leaf ? lvl.isFree(p) : lvl.isFreeBound(p))) {
      score += 1;  // unknown: neither occupied nor known-free.
    }
    // else: known-free, contributes 0.
    if (point_tests_out != nullptr) ++(*point_tests_out);

    const int remaining = n - i - 1;
    if (score + remaining * per_point_max < min_required) return score;  // cannot possibly reach min_required.
  }
  return score;
}

/// @brief Per-point breakdown of a score, for diagnostics only. Meaning of each field
/// depends on the mode it was computed under (see scoreBreakdownAtLevel()).
struct ScoreBreakdown
{
  int hits = 0;  ///< Occupancy: points landing in occupied (or hitBound-occupied) space.
                 ///< DistanceField: points whose cell value is non-zero, i.e. within
                 ///< `df_truncation_voxels` of some occupied voxel.
  int unknown = 0;  ///< Occupancy: points in neither occupied nor known-free space.
                 ///< DistanceField: points whose cell value is 0 (absent from the field).
  int free = 0;  ///< Occupancy: points landing in known-free space. Always 0 under
                 ///< DistanceField -- that mode ignores the free-space channel entirely.
  int raw = 0;  ///< The score scorePoseAtLevel() returns for identical arguments:
                ///< `hit_weight * hits + unknown` under Occupancy, the sum of per-point
                ///< cell values under DistanceField.
  int max_possible = 0;  ///< Upper bound on `raw`: `hit_weight * n` under Occupancy,
                ///< `255 * n` under DistanceField -- lets a caller normalise `raw`
                ///< without knowing which mode produced it.
  double mean_cell_score = 0.0;  ///< `raw / n`, for diagnostics: the average per-point
                ///< contribution, regardless of mode.
};

/**
 * @brief Exhaustively break a score down into its per-point components.
 *
 * Diagnostics only: unlike scorePoseAtLevel(), this always tests every query
 * point (no `min_required` early exit), so callers can log the composition of
 * a score without perturbing search performance. Uses the exact same per-point
 * classification as scorePoseAtLevel(), so `breakdown.raw` always equals what
 * `scorePoseAtLevel(..., level, hit_weight, 0, nullptr, score_mode)` returns
 * for identical arguments.
 *
 * @param pyramid Pyramid to score against.
 * @param query Body-frame, gravity-de-tilted query points.
 * @param translation Candidate map-frame translation.
 * @param yaw Candidate map-frame yaw about Z (rad).
 * @param level Pyramid level to score against, 0 is finest.
 * @param hit_weight Per-point weight W for an occupied hit; see scorePoseAtLevel(). Only
 *   consulted under `ScoreMode::Occupancy`.
 * @param score_mode Scoring model to evaluate; see scorePoseAtLevel().
 * @return The per-point breakdown and the resulting raw score.
 */
inline ScoreBreakdown scoreBreakdownAtLevel(
  const VoxelPyramid & pyramid, const std::vector<Eigen::Vector3d> & query,
  const Eigen::Vector3d & translation, double yaw, int level, int hit_weight = 3,
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

/**
 * @brief One frontier node in the branch-and-bound search.
 *
 * `bound` is the score computed by `scorePoseAtLevel()` at `level`: an upper
 * bound for `level > 0`, and the exact leaf score for `level == 0`.
 */
struct BnbNode
{
  int level = 0;  ///< Pyramid level this node lives at, 0 is finest.
  int64_t ix = 0;  ///< Voxel index along x at `level`.
  int64_t iy = 0;  ///< Voxel index along y at `level`.
  int64_t iz = 0;  ///< Voxel index along z at `level`.
  int64_t yaw_bin = 0;  ///< Yaw bin index at `level`.
  int bound = 0;  ///< Upper bound (or exact leaf score) for this node.
};

/// @brief Orders `BnbNode`s so the priority queue pops the highest bound first,
/// breaking ties by depth.
struct BnbNodeGreaterByBound
{
  /// @param a Left-hand node.
  /// @param b Right-hand node.
  /// @return True if `a` should sort below `b`: either `a`'s bound is smaller,
  ///   or the bounds are equal and `a` is the shallower (larger-level) node.
  ///
  /// Bounds are small integers and tie constantly, so on a tie the DEEPER node
  /// (smaller `level`) is popped first -- `a.level > b.level` sorts `a` below
  /// `b`. This pushes the search depth-first through tied regions, raising the
  /// incumbent earlier so the prune threshold actually bites sooner. It is
  /// still an exact best-first search: only the tie-break order changes, never
  /// which nodes are admissible.
  bool operator()(const BnbNode & a, const BnbNode & b) const
  {
    if (a.bound != b.bound) return a.bound < b.bound;
    return a.level > b.level;
  }
};

/**
 * @brief Best-first branch-and-bound search for the 4-DOF pose maximizing hits.
 *
 * Seeds the frontier with every root cell crossed with every coarsest-level
 * yaw bin, then repeatedly pops the highest-bound node: level-0 leaves are
 * recorded as solutions, and internal nodes expand into 16 children (8
 * translation children x 2 yaw children) whose bounds exceed the current
 * prune threshold. See `SearchConfig::prune_slack` for why that threshold sits
 * below `best_score` rather than at it, and `bnbCellCentre()` for why node
 * poses are cell-centre / bin-centre rather than corner-based.
 *
 * @param pyramid Occupancy pyramid to search against.
 * @param query Body-frame, gravity-de-tilted query points.
 * @param roots Starting cells at the pyramid's coarsest level.
 * @param cfg Search tuning parameters.
 * @param stats Output: counters describing this run.
 * @return Up to `cfg.max_solutions` hypotheses, sorted by score descending and
 *   mutually separated by more than `cfg.nms_radius`.
 */
inline std::vector<Hypothesis> branchAndBound(
  const VoxelPyramid & pyramid, const std::vector<Eigen::Vector3d> & query, const std::vector<RootCell> & roots,
  const SearchConfig & cfg, SearchStats & stats)
{
  stats = SearchStats{};
  std::vector<Hypothesis> solutions;
  if (query.empty() || roots.empty() || pyramid.empty() || pyramid.numLevels() <= 0) return solutions;

  const YawDiscretization yaw_disc = YawDiscretization::compute(query, pyramid);
  if (yaw_disc.max_range <= 0.0) return solutions;  // degenerate query set, guard div-by-zero.

  const int coarsest = pyramid.numLevels() - 1;

  // Bound/score a node's representative pose (cell centre, bin centre) against
  // its own level, via the single shared scoring routine. `min_required` is
  // forwarded to `scorePoseAtLevel()`'s early exit -- see the soundness
  // invariant documented there: a value this returns that is `> min_required - 1`
  // is always exact, never truncated, so it is safe to push onto the frontier.
  auto boundOf = [&](int level, int64_t ix, int64_t iy, int64_t iz, int64_t yaw_bin, int min_required) -> int {
    const double resolution = pyramid.level(level).resolution;
    const Eigen::Vector3d translation = bnbCellCentre(ix, iy, iz, resolution);
    const double yaw = yaw_disc.binCentre(level, yaw_bin);
    return scorePoseAtLevel(
      pyramid, query, translation, yaw, level, cfg.hit_weight, min_required, &stats.point_tests, cfg.score_mode);
  };

  // Build the Hypothesis for a level-0 leaf, including the hits/hit_fraction breakdown
  // that scorePoseAtLevel()'s single ternary count does not expose. Exhaustive (via
  // scoreBreakdownAtLevel(), which never early-exits), but only ever called once per
  // accepted leaf, not per node -- negligible next to the bound evaluations above.
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
    // max_possible = hit_weight*n or 255*n, per breakdown's own mode -- see ScoreBreakdown.
    h.normalized = breakdown.max_possible > 0
                     ? static_cast<double>(breakdown.raw) / static_cast<double>(breakdown.max_possible)
                     : 0.0;
    return h;
  };

  std::priority_queue<BnbNode, std::vector<BnbNode>, BnbNodeGreaterByBound> frontier;

  int best_score = 0;
  // Starts at -1 (rather than 0) so every non-negative bound is admissible
  // before any solution has been found -- and so `prune_threshold + 1 == 0`,
  // under which scorePoseAtLevel()'s early exit can never fire (see its
  // default-argument doc), keeping this seeding pass exhaustive per node.
  int prune_threshold = -1;

  const int64_t n_coarse = yaw_disc.numBins(coarsest);
  for (const auto & root : roots) {
    for (int64_t k = 0; k < n_coarse; ++k) {
      // A node is kept only when its bound exceeds prune_threshold, exactly as
      // for child expansion below; see the soundness invariant on scorePoseAtLevel().
      const int bound = boundOf(coarsest, root.ix, root.iy, root.iz, k, prune_threshold + 1);
      if (bound > prune_threshold) {
        frontier.push(BnbNode{coarsest, root.ix, root.iy, root.iz, k, bound});
      } else {
        ++stats.nodes_pruned;
      }
    }
  }

  // Greedy dive: establish a real incumbent before the main loop runs, so the
  // threshold-aware early exit in scorePoseAtLevel() (Change 1) has something
  // to bite on from the very first pop instead of only after ~48k blind
  // expansions. Starting from the single highest-bound root node (the
  // frontier's current top, under BnbNodeGreaterByBound's max-heap ordering),
  // repeatedly evaluate all 16 children and follow only the best-scoring one
  // down to a level-0 leaf. Every evaluation here uses min_required = 0, so
  // per the soundness invariant none of these scores are ever truncated --
  // this dive is exact, not part of the pruned search tree, and does not
  // affect nodes_expanded/nodes_pruned.
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

    // cur is now a level-0 leaf, scored exactly (min_required = 0 above).
    const Hypothesis h = leafHypothesis(cur.ix, cur.iy, cur.iz, cur.yaw_bin);
    solutions.push_back(h);

    best_score = h.score;
    // Same slack rule as the main loop's incumbent update below.
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

    // The frontier is ordered by bound descending, so once the best remaining
    // node cannot beat the threshold, nothing behind it can either.
    if (node.bound <= prune_threshold) break;

    if (node.level == 0) {
      // node.bound is the exact leaf score already (scorePoseAtLevel() used hit()/
      // isFree() at level 0); leafHypothesis() recomputes it via scoreBreakdownAtLevel()
      // to also get the hits/hit_fraction split -- the two must and do agree.
      const Hypothesis h = leafHypothesis(node.ix, node.iy, node.iz, node.yaw_bin);
      solutions.push_back(h);

      if (h.score > best_score) {
        best_score = h.score;
        // Textbook BnB would prune at best_score; see SearchConfig::prune_slack
        // for why we deliberately prune below it instead.
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
            // A node is kept only when its bound exceeds prune_threshold, so the
            // score floor passed in is exactly one above that threshold. Per the
            // soundness invariant on scorePoseAtLevel(): child_bound can only be
            // truncated when it is < min_required, i.e. <= prune_threshold, i.e.
            // exactly the branch that gets pruned below -- so a child_bound that
            // passes the `> prune_threshold` test here was always scored to
            // completion and is exact, never a truncated partial count.
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

/**
 * @brief Exhaustively score every root cell x every coarsest-level yaw bin.
 *
 * No pruning: every candidate is scored via the same `scorePoseAtLevel()`
 * routine (against the coarsest level, using `hitBound()`) that seeds
 * `branchAndBound()`'s frontier, so this validates the scoring function
 * independently of the bound-and-prune logic.
 *
 * @param pyramid Pyramid to score against.
 * @param query Body-frame, gravity-de-tilted query points.
 * @param roots Starting cells at the pyramid's coarsest level.
 * @param max_results Maximum number of hypotheses to return.
 * @param score_mode Scoring model to evaluate; see scorePoseAtLevel(). Must match the
 *   mode the caller wants to validate against `branchAndBound()` -- passing the wrong
 *   mode here scores against a channel the pyramid may never have built (e.g.
 *   DistanceField against a pyramid built with `score_mode = Occupancy` reads an empty
 *   score grid and returns all zeros).
 * @return Up to `max_results` hypotheses, sorted by score descending.
 */
inline std::vector<Hypothesis> bruteForceCoarse(
  const VoxelPyramid & pyramid, const std::vector<Eigen::Vector3d> & query, const std::vector<RootCell> & roots,
  int max_results, ScoreMode score_mode = ScoreMode::DistanceField)
{
  std::vector<Hypothesis> results;
  if (query.empty() || roots.empty() || pyramid.empty() || pyramid.numLevels() <= 0) return results;

  const YawDiscretization yaw_disc = YawDiscretization::compute(query, pyramid);
  if (yaw_disc.max_range <= 0.0) return results;

  const int coarsest = pyramid.numLevels() - 1;
  const double resolution = pyramid.level(coarsest).resolution;
  const int64_t n_coarse = yaw_disc.numBins(coarsest);
  // No SearchConfig here, so use scorePoseAtLevel()/scoreBreakdownAtLevel()'s own default
  // hit_weight -- the same default as SearchConfig::hit_weight, so a caller comparing
  // this oracle against branchAndBound() at default settings gets a matching scale.
  // Only consulted under ScoreMode::Occupancy.
  constexpr int kHitWeight = 3;

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
      h.normalized = breakdown.max_possible > 0
                       ? static_cast<double>(h.score) / static_cast<double>(breakdown.max_possible)
                       : 0.0;
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

/**
 * @brief Exhaustively score every level-0 cell in a box x every level-0 yaw bin.
 *
 * Reference oracle for branch-and-bound equivalence testing: every level-0
 * cell whose centre falls in `[box_min, box_max]` is scored against every
 * level-0 yaw bin, using the same `scorePoseAtLevel()` / cell-centre / bin-centre
 * convention as the branch-and-bound leaf. Intentionally slow (cubic in box
 * size times query size times yaw bins); only suitable for small boxes in tests.
 *
 * @param pyramid Pyramid to score against.
 * @param query Body-frame, gravity-de-tilted query points.
 * @param box_min Map-frame lower corner of the search box.
 * @param box_max Map-frame upper corner of the search box.
 * @param max_results Maximum number of hypotheses to return.
 * @param score_mode Scoring model to evaluate; see bruteForceCoarse() for why this must
 *   match the mode the pyramid was actually built with.
 * @return Up to `max_results` hypotheses, sorted by score descending.
 */
inline std::vector<Hypothesis> bruteForceLeaf(
  const VoxelPyramid & pyramid, const std::vector<Eigen::Vector3d> & query, const Eigen::Vector3d & box_min,
  const Eigen::Vector3d & box_max, int max_results, ScoreMode score_mode = ScoreMode::DistanceField)
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
  // See bruteForceCoarse() for why this matches SearchConfig::hit_weight's default.
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
