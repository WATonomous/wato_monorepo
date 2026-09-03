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
#include <climits>  // for INT64_MIN, used as VoxelScoreGrid's empty-slot sentinel
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef __GLIBC__
#include <malloc.h>  // for malloc_trim(), used by VoxelPyramid::releaseMemory()
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eidos::reloc
{

/// @brief Bits allocated per axis in a packed voxel key.
constexpr int kVoxelBits = 21;
/// @brief Bias added to signed voxel indices so they pack as unsigned.
constexpr int64_t kVoxelBias = 1LL << (kVoxelBits - 1);
/// @brief Mask for one axis of a packed voxel key.
constexpr int64_t kVoxelMask = (1LL << kVoxelBits) - 1;
/// @brief Largest magnitude voxel index representable per axis.
constexpr int64_t kVoxelIndexLimit = kVoxelBias - 1;

/**
 * @brief Pack signed voxel indices into a single 64-bit key.
 *
 * Uses 21 bits per axis with a bias, so indices in [-2^20, 2^20-1] are
 * representable. At the default 1 m resolution that is +/- 1048 km, far beyond
 * any plausible map extent.
 *
 * @param ix Voxel index along x.
 * @param iy Voxel index along y.
 * @param iz Voxel index along z.
 * @return Packed key.
 */
inline int64_t packVoxel(int64_t ix, int64_t iy, int64_t iz)
{
  return ((ix + kVoxelBias) & kVoxelMask) | (((iy + kVoxelBias) & kVoxelMask) << kVoxelBits) |
         (((iz + kVoxelBias) & kVoxelMask) << (2 * kVoxelBits));
}

/**
 * @brief Unpack a voxel key back into signed indices.
 *
 * @param key Packed key.
 * @param ix Output index along x.
 * @param iy Output index along y.
 * @param iz Output index along z.
 */
inline void unpackVoxel(int64_t key, int64_t & ix, int64_t & iy, int64_t & iz)
{
  ix = (key & kVoxelMask) - kVoxelBias;
  iy = ((key >> kVoxelBits) & kVoxelMask) - kVoxelBias;
  iz = ((key >> (2 * kVoxelBits)) & kVoxelMask) - kVoxelBias;
}

/**
 * @brief Floor division of a coordinate by a resolution, as an integer index.
 *
 * std::floor is used rather than a cast because casts truncate toward zero,
 * which would fold negative coordinates onto the wrong voxel.
 *
 * @param v Coordinate value.
 * @param inv_resolution Reciprocal of the voxel resolution.
 * @return Voxel index.
 */
inline int64_t voxelIndex(double v, double inv_resolution)
{
  return static_cast<int64_t>(std::floor(v * inv_resolution));
}

/// @brief Integer hash for packed voxel keys (splitmix64 finalizer).
struct VoxelHash
{
  /// @brief Hash a packed voxel key.
  /// @param k Packed key.
  /// @return Hash value.
  std::size_t operator()(int64_t k) const noexcept
  {
    uint64_t x = static_cast<uint64_t>(k) + 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return static_cast<std::size_t>(x ^ (x >> 31));
  }
};

using VoxelSet = std::unordered_set<int64_t, VoxelHash>;

/**
 * @brief Selects which per-point scoring model `VoxelPyramid`/`bnb_search.hpp` build and
 * evaluate against.
 *
 * `Occupancy` reproduces the original ternary score bit-for-bit: occupied/hitBound
 * contributes `hit_weight`, known-free contributes 0, unknown contributes 1 (see
 * `scorePoseAtLevel()`). `DistanceField` is the new default: each level-0 cell holds a
 * continuous falloff value in `[0, 255]` around the nearest occupied voxel, and a pose
 * score is the sum of per-query-point cell values. Keeping both modes live is what makes
 * it possible to A/B the two scorers against the same map and the same scan -- see
 * CONTRACT_DF.md for the measurement that motivated the distance field in the first
 * place (binary containment carries almost no alignment signal on a vegetated scene).
 */
enum class ScoreMode
{
  Occupancy,
  DistanceField
};

/**
 * @brief Compact voxel -> score map for the distance-field channel.
 *
 * Open addressing, power-of-two capacity, linear probing, held in TWO PARALLEL FLAT
 * VECTORS (keys, values) rather than the per-element nodes an `unordered_set` (i.e.
 * `VoxelSet`) allocates. That is the whole reason this class exists instead of reusing
 * `VoxelSet` the way `VoxelLevel::voxels`/`free_voxels` do: freeing millions of
 * `unordered_set` nodes returns them to the glibc arena but typically NOT to the OS, so
 * RSS stays high across a relocalization lock -- exactly the failure mode the caller
 * needs memory to recover from. Two large contiguous blocks are what let `clear()`
 * actually hand memory back to the allocator (see its swap-with-empty idiom below), and
 * the cache locality of a flat probe sequence also makes the lookup in the scoring hot
 * path considerably cheaper than chasing an `unordered_set` node.
 */
class VoxelScoreGrid
{
public:
  /// @brief Sentinel for an unoccupied slot. Voxel keys are biased-packed coordinates --
  /// 3 axes x kVoxelBits = 63 bits, always non-negative (see packVoxel()) -- so
  /// INT64_MIN is never a legal key and is safe to use as the empty marker.
  static constexpr int64_t kEmptyKey = INT64_MIN;

  /// @brief Load factor above which the table doubles in capacity.
  static constexpr double kMaxLoadFactor = 0.6;

  /**
   * @brief Size the table up front for at least `expected_elements` entries at
   * `<= kMaxLoadFactor`, avoiding incremental rehashes during a known-size build.
   * @param expected_elements Number of entries the caller expects to insert.
   */
  void reserve(std::size_t expected_elements)
  {
    if (expected_elements == 0) return;
    const std::size_t needed =
      static_cast<std::size_t>(std::ceil(static_cast<double>(expected_elements) / kMaxLoadFactor));
    const std::size_t target = roundUpPow2(std::max<std::size_t>(needed, kInitialCapacity));
    if (target > capacity_) rehash(target);
  }

  /**
   * @brief `grid[key] = max(grid[key], score)`, inserting `key` if it was absent.
   * @param key Packed voxel key. Must not equal kEmptyKey.
   * @param score Candidate score for this voxel.
   */
  void maxInsert(int64_t key, uint8_t score)
  {
    if (capacity_ == 0) rehash(kInitialCapacity);
    const std::size_t idx = slotFor(key);
    if (keys_[idx] == kEmptyKey) {
      keys_[idx] = key;
      values_[idx] = score;
      ++size_;
      // Grow strictly after inserting, never before: the slot just written is still
      // valid in the OLD table, and rehash() below re-probes every entry (including
      // this one) into the new one.
      if (static_cast<double>(size_) > kMaxLoadFactor * static_cast<double>(capacity_)) {
        rehash(capacity_ * 2);
      }
    } else if (score > values_[idx]) {
      values_[idx] = score;
    }
  }

  /**
   * @brief Look up a voxel's score.
   * @param key Packed voxel key.
   * @return Stored score, or 0 if `key` is absent -- absence and a stored zero are
   *   deliberately indistinguishable to callers (see the module contract's "Absent ==
   *   score 0").
   */
  uint8_t at(int64_t key) const
  {
    if (capacity_ == 0) return 0;
    const std::size_t idx = slotFor(key);
    return keys_[idx] == kEmptyKey ? static_cast<uint8_t>(0) : values_[idx];
  }

  /// @brief Number of stored entries.
  /// @return Entry count.
  std::size_t size() const
  {
    return size_;
  }

  /// @brief Current table capacity (slot count; a power of two).
  /// @return Capacity.
  std::size_t capacity() const
  {
    return capacity_;
  }

  /**
   * @brief Exact heap footprint of the two backing vectors.
   * @return `capacity() * (sizeof(int64_t) + sizeof(uint8_t))`. This is EXACT, not an
   *   estimate -- unlike `VoxelLevel::memoryBytes()`'s hash-set accounting, this class's
   *   entire footprint is two flat vectors sized to their own capacity, so there is
   *   nothing left to approximate.
   */
  std::size_t memoryBytes() const
  {
    return capacity_ * (sizeof(int64_t) + sizeof(uint8_t));
  }

  /// @brief Whether the grid holds no entries.
  /// @return True if empty.
  bool empty() const
  {
    return size_ == 0;
  }

  /**
   * @brief Release all capacity back to the allocator.
   *
   * Uses the swap-with-empty idiom rather than `keys_.clear(); values_.clear();`:
   * `vector::clear()` destroys the elements but keeps the allocated buffer, which would
   * silently defeat the entire reason this class exists over a `VoxelSet`. Swapping with
   * a default-constructed (zero-capacity) vector hands the old buffer to a temporary
   * that is freed the moment this function returns.
   */
  void clear()
  {
    std::vector<int64_t>().swap(keys_);
    std::vector<uint8_t>().swap(values_);
    capacity_ = 0;
    size_ = 0;
  }

  /**
   * @brief Visit every stored (key, score) pair. Skips empty slots.
   * @param f Callable as `f(int64_t key, uint8_t score)`.
   */
  template <class F>
  void forEach(F && f) const
  {
    for (std::size_t i = 0; i < capacity_; ++i) {
      if (keys_[i] != kEmptyKey) f(keys_[i], values_[i]);
    }
  }

private:
  static constexpr std::size_t kInitialCapacity = 16;

  /// @brief Smallest power of two `>= v` (returns 1 for v == 0).
  static std::size_t roundUpPow2(std::size_t v)
  {
    std::size_t p = 1;
    while (p < v) p <<= 1;
    return p;
  }

  /// @brief Find `key`'s slot via linear probing, or the first empty slot on its probe
  /// sequence if `key` is not present. Requires `capacity_ > 0`.
  std::size_t slotFor(int64_t key) const
  {
    const std::size_t mask = capacity_ - 1;
    std::size_t idx = VoxelHash{}(key) & mask;
    while (keys_[idx] != kEmptyKey && keys_[idx] != key) {
      idx = (idx + 1) & mask;
    }
    return idx;
  }

  /// @brief Grow to `new_capacity` (rounded up to a power of two) and re-probe every
  /// existing entry into the new table. Never called with a capacity that would drop
  /// any entry -- callers only ever grow.
  void rehash(std::size_t new_capacity)
  {
    new_capacity = roundUpPow2(std::max<std::size_t>(new_capacity, kInitialCapacity));
    std::vector<int64_t> new_keys(new_capacity, kEmptyKey);
    std::vector<uint8_t> new_values(new_capacity, 0);
    const std::size_t mask = new_capacity - 1;
    for (std::size_t i = 0; i < capacity_; ++i) {
      if (keys_[i] == kEmptyKey) continue;
      std::size_t idx = VoxelHash{}(keys_[i]) & mask;
      while (new_keys[idx] != kEmptyKey) idx = (idx + 1) & mask;
      new_keys[idx] = keys_[i];
      new_values[idx] = values_[i];
    }
    keys_.swap(new_keys);
    values_.swap(new_values);
    capacity_ = new_capacity;
  }

  std::vector<int64_t> keys_;
  std::vector<uint8_t> values_;
  std::size_t capacity_ = 0;
  std::size_t size_ = 0;
};

/**
 * @brief One resolution level of the occupancy pyramid.
 *
 * For non-leaf levels the stored set is normally pre-dilated by its
 * 26-neighbourhood so that a single lookup yields a valid branch-and-bound
 * upper bound. When dilation would exceed the memory budget, the exact set is
 * kept instead and `dilated` is false; `hitBound()` then probes the
 * 27-neighbourhood on the fly, trading time for memory.
 */
struct VoxelLevel
{
  double resolution = 1.0;  ///< Edge length of a voxel at this level (m).
  double inv_resolution = 1.0;  ///< Cached 1 / resolution.
  VoxelSet voxels;  ///< Occupied set, dilated when `dilated` is true.
  bool dilated = false;  ///< Whether `voxels` already includes the 26-neighbourhood.
  VoxelSet free_voxels;  ///< Known-empty set. Level 0: exact. Level>0: conservative, eroded by
                          ///< finalize() so a lookup here is sound as a bound (see isFreeBound()).
  bool has_free = false;  ///< Whether `free_voxels` is populated at this level. When false,
                          ///< `isFreeBound()` always returns false, never true.
  VoxelScoreGrid scores;  ///< Distance-field values (see ScoreMode::DistanceField). Empty in
                          ///< Occupancy mode. Level 0: exact, built by max-dilating a falloff
                          ///< kernel around every occupied voxel. Level>0: max-pooled over its 8
                          ///< children then max-dilated over its own 26-neighbourhood, so, exactly
                          ///< like the dilated `voxels` set, a single lookup is already a valid
                          ///< upper bound -- see VoxelPyramid::buildDistanceField().

  /**
   * @brief Exact occupancy test for a point.
   * @param p Point in map frame.
   * @return True if the containing voxel is in the set.
   */
  bool hit(const Eigen::Vector3d & p) const
  {
    return voxels.find(packVoxel(
             voxelIndex(p.x(), inv_resolution),
             voxelIndex(p.y(), inv_resolution),
             voxelIndex(p.z(), inv_resolution))) != voxels.end();
  }

  /**
   * @brief Upper-bound occupancy test for a point.
   *
   * Equivalent to testing the 26-neighbourhood-dilated set. When the set is
   * already dilated this is a single probe; otherwise 27 probes are issued.
   *
   * @param p Point in map frame.
   * @return True if the containing voxel or any neighbour is occupied.
   */
  bool hitBound(const Eigen::Vector3d & p) const
  {
    const int64_t vx = voxelIndex(p.x(), inv_resolution);
    const int64_t vy = voxelIndex(p.y(), inv_resolution);
    const int64_t vz = voxelIndex(p.z(), inv_resolution);
    if (dilated) {
      return voxels.find(packVoxel(vx, vy, vz)) != voxels.end();
    }
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (voxels.find(packVoxel(vx + dx, vy + dy, vz + dz)) != voxels.end()) return true;
        }
      }
    }
    return false;
  }

  /**
   * @brief Exact known-free test for a point.
   *
   * Meaningful at level 0, the leaf: `free_voxels` there is the exact raycast result
   * (after finalize()'s occupancy-wins erasure), so this is a plain membership test,
   * the free-space mirror of hit().
   *
   * @param p Point in map frame.
   * @return True if the containing voxel is in the free set.
   */
  bool isFree(const Eigen::Vector3d & p) const
  {
    return free_voxels.find(packVoxel(
             voxelIndex(p.x(), inv_resolution),
             voxelIndex(p.y(), inv_resolution),
             voxelIndex(p.z(), inv_resolution))) != free_voxels.end();
  }

  /**
   * @brief Conservative known-free test for bound computation at level > 0.
   *
   * True only if the point is free at level 0 for EVERY pose a descendant of this
   * node can take. finalize() builds this by eroding the level's fully-free set by
   * its 26-neighbourhood -- the inverse move of hitBound()'s dilation: occupied is
   * inflated to over-estimate hits, free is shrunk to under-estimate penalties, so
   * that a coarse "definitely free" reading can never be wrong about a finer pose.
   * `has_free` gates this explicitly, so a level with no free data is never mistaken
   * for one that is free everywhere. At level 0 this is identical to isFree().
   *
   * @param p Point in map frame.
   * @return True if the point is free for every descendant pose, false if unknown.
   */
  bool isFreeBound(const Eigen::Vector3d & p) const
  {
    if (!has_free) return false;
    return isFree(p);
  }

  /**
   * @brief Exact level-0 distance-field lookup.
   * @param p Point in map frame.
   * @return Stored falloff value in `[0, 255]`, or 0 if the voxel is absent from the
   *   field (beyond `df_truncation_voxels` of every occupied voxel).
   */
  uint8_t scoreAt(const Eigen::Vector3d & p) const
  {
    return scores.at(packVoxel(
      voxelIndex(p.x(), inv_resolution), voxelIndex(p.y(), inv_resolution), voxelIndex(p.z(), inv_resolution)));
  }

  /**
   * @brief Coarse distance-field bound for level > 0.
   *
   * Levels above 0 are stored pre-max-pooled and pre-max-dilated by
   * VoxelPyramid::buildDistanceField(), so a single lookup here is already a valid upper
   * bound on every descendant's exact level-0 value -- unlike `hitBound()`, there is no
   * on-the-fly neighbourhood-probing fallback to fall into, because the distance field
   * has no equivalent of `dilationSkipped()`: buildDistanceField() either completes the
   * full pyramid or abandons the field entirely (see `max_score_voxels`), never leaves
   * one level partially built. At level 0 this is identical to scoreAt().
   *
   * @param p Point in map frame.
   * @return Stored falloff value in `[0, 255]`, or 0 if absent from the field.
   */
  uint8_t scoreBound(const Eigen::Vector3d & p) const
  {
    return scoreAt(p);
  }

  /// @brief Approximate heap footprint of this level in bytes.
  /// @return Estimated bytes.
  std::size_t memoryBytes() const
  {
    // unordered_set of int64: one node (key + next pointer) per element plus the
    // bucket array. This is an estimate, not an exact accounting. The free set is
    // counted too -- it is a VOLUME rather than a surface, so on an open site it can
    // rival or exceed the occupied set, and a memory figure that omitted it would
    // understate the pyramid badly at exactly the sizes where that matters. `scores`,
    // by contrast, reports its OWN exact footprint (see VoxelScoreGrid::memoryBytes()),
    // so it is simply added rather than approximated here.
    const auto set_bytes = [](const VoxelSet & s) {
      return s.size() * (sizeof(int64_t) + sizeof(void *)) + s.bucket_count() * sizeof(void *);
    };
    return set_bytes(voxels) + set_bytes(free_voxels) + scores.memoryBytes();
  }
};

/**
 * @brief Multi-resolution sparse occupancy pyramid over a prior point cloud map.
 *
 * Level 0 is the finest (`min_voxel_size`) and is kept undilated because it is
 * the branch-and-bound leaf level, where the exact score is wanted. Coarser
 * levels double in resolution and are dilated to serve as upper bounds.
 *
 * Coarse levels are derived from the level-0 index set rather than from the raw
 * points. Because resolutions are power-of-two multiples anchored at the origin,
 * a level-l voxel is exactly the union of the level-0 voxels inside it, so the
 * two constructions are identical while the index-set route is far cheaper.
 */
class VoxelPyramid
{
public:
  /// @brief Pyramid construction parameters.
  struct Config
  {
    double min_voxel_size = 1.0;  ///< Finest voxel edge length (m).
    int num_levels = 6;  ///< Number of resolution levels.
    double max_height = 6.0;  ///< Discard points above this map-frame z; <= 0 disables.
    std::size_t max_voxels_per_level = 8000000;  ///< Dilation budget per level.

    bool build_free_space = false;  ///< Master switch. false => byte-identical to today's pyramid.
    double free_max_range = 40.0;  ///< Clamp ray length (m). Bounds raycast cost.
    double free_end_margin = 1.0;  ///< Stop the ray this many metres short of its endpoint, so the
                                    ///< surface itself is never marked free.
    double free_min_height = 0.0;  ///< Band on (voxel z - ray origin z). Both 0 disables the band.
    double free_max_height = 0.0;
    bool free_clear_near_occupied = true;  ///< At finalize, delete every level-0 free voxel that is
                                    ///< occupied OR 26-adjacent to an occupied voxel. Makes free
                                    ///< space "confidently free" so registration jitter and
                                    ///< vegetation near real structure are not penalised.
    std::size_t max_free_voxels = 20000000;  ///< Budget guard; on overflow abandon free space
                                    ///< entirely (clear all free sets, has_free=false everywhere)
                                    ///< rather than build a partial channel. Loosening the bound
                                    ///< this way is always sound; see freeSpaceAbandoned().

    ScoreMode score_mode = ScoreMode::DistanceField;  ///< New default; see ScoreMode. Under
                                    ///< DistanceField the free-space channel above is ignored
                                    ///< entirely -- insertRay() and buildFreeSpace() both become
                                    ///< no-ops -- because a continuous falloff already scores a
                                    ///< point far from all structure near zero, which is what the
                                    ///< ternary free state was approximating.
    double df_sigma = 1.0;         ///< Metres; the Gaussian falloff's std-dev, i.e. roughly the
                                    ///< registration offset the field is meant to absorb gracefully
                                    ///< rather than threshold to zero (measured ~1 m on ring_road.map,
                                    ///< see CONTRACT_DF.md).
    int df_truncation_voxels = 2;  ///< Kernel radius in level-0 voxels: cells further than this
                                    ///< (in Chebyshev distance, from the kernel construction's own
                                    ///< cube iteration -- see buildDistanceField()) from every
                                    ///< occupied voxel are not stored, i.e. score 0.
    std::size_t max_score_voxels = 40000000;  ///< Budget guard on the total number of stored
                                    ///< distance-field entries across all levels; on overflow
                                    ///< abandon the field entirely and fall back to Occupancy mode
                                    ///< for this build (see distanceFieldAbandoned(),
                                    ///< effectiveScoreMode()) rather than build a partial field.
  };

  /**
   * @brief Reset the pyramid and begin accumulating points at a given resolution.
   * @param cfg Construction parameters.
   */
  void beginInsert(const Config & cfg)
  {
    cfg_ = cfg;
    levels_.clear();
    levels_.resize(1);
    levels_[0].resolution = cfg.min_voxel_size;
    levels_[0].inv_resolution = 1.0 / cfg.min_voxel_size;
    levels_[0].dilated = false;
    dilation_skipped_.assign(static_cast<std::size_t>(cfg.num_levels), false);
    out_of_range_points_ = 0;
    free_space_abandoned_ = false;
    distance_field_abandoned_ = false;
  }

  /**
   * @brief Insert one map-frame point into the finest level.
   *
   * Points above `max_height` are discarded. Points whose voxel index exceeds
   * the packing range are counted in `outOfRangePoints()` and dropped rather
   * than silently aliasing onto a wrong voxel.
   *
   * @param p Point in map frame.
   */
  void insert(const Eigen::Vector3d & p)
  {
    if (cfg_.max_height > 0.0 && p.z() > cfg_.max_height) return;
    const double inv = levels_[0].inv_resolution;
    const int64_t ix = voxelIndex(p.x(), inv);
    const int64_t iy = voxelIndex(p.y(), inv);
    const int64_t iz = voxelIndex(p.z(), inv);
    if (
      std::abs(ix) > kVoxelIndexLimit || std::abs(iy) > kVoxelIndexLimit ||
      std::abs(iz) > kVoxelIndexLimit) {
      ++out_of_range_points_;
      return;
    }
    levels_[0].voxels.insert(packVoxel(ix, iy, iz));
  }

  /**
   * @brief Mark voxels along `origin -> endpoint` as known-free at level 0.
   *
   * No-op unless `cfg_.build_free_space` (or once the free-space budget has already
   * been abandoned this build). Must be called between beginInsert() and finalize();
   * not thread-safe, same contract as insert().
   *
   * Walks the ray with the standard Amanatides-Woo 3D grid traversal: at each step the
   * axis whose parametric distance to its next voxel boundary (`tMax`) is smallest is
   * advanced, so every voxel the ray actually passes through is visited exactly once,
   * in order, with no skips. A zero (or numerically negligible) direction component
   * would divide by zero computing that axis's `tMax`/`tDelta`, so such axes are given
   * an infinite `tMax` instead -- they simply never win the "smallest tMax" comparison,
   * which is exactly correct since the ray never crosses a boundary on that axis.
   *
   * The number of voxels to mark is computed up front from the (clamped) start and end
   * voxel indices -- a plain integer Manhattan distance, not a floating-point marching
   * condition -- so the loop below is bounded by construction and cannot spin forever
   * regardless of any tMax accumulation drift.
   *
   * Also a no-op under `cfg_.score_mode == ScoreMode::DistanceField`: that mode ignores
   * the free-space channel entirely (see ScoreMode / Config::score_mode), so building it
   * would only spend the (non-trivial) DDA cost on a channel scorePoseAtLevel() never
   * consults.
   *
   * @param origin Ray origin in map frame (typically the sensor position).
   * @param endpoint Ray endpoint in map frame (typically a returned range point).
   */
  void insertRay(const Eigen::Vector3d & origin, const Eigen::Vector3d & endpoint)
  {
    if (!cfg_.build_free_space || free_space_abandoned_ || cfg_.score_mode == ScoreMode::DistanceField) return;

    const Eigen::Vector3d delta = endpoint - origin;
    const double length = delta.norm();
    // A zero-length (or numerically negligible) ray has no interior to mark free, and
    // dividing by `length` below would be undefined.
    constexpr double kMinRayLength = 1e-6;
    if (length < kMinRayLength) return;

    const double clamped_length = std::min(length, cfg_.free_max_range);
    const Eigen::Vector3d dir = delta / length;  // unit direction
    const Eigen::Vector3d clamped_end = origin + dir * clamped_length;

    VoxelLevel & lvl0 = levels_[0];
    const double r0 = lvl0.resolution;
    const double inv0 = lvl0.inv_resolution;

    int64_t ix = voxelIndex(origin.x(), inv0);
    int64_t iy = voxelIndex(origin.y(), inv0);
    int64_t iz = voxelIndex(origin.z(), inv0);
    const int64_t ix_end = voxelIndex(clamped_end.x(), inv0);
    const int64_t iy_end = voxelIndex(clamped_end.y(), inv0);
    const int64_t iz_end = voxelIndex(clamped_end.z(), inv0);

    // Exact number of voxel-to-voxel steps from the start voxel to the (clamped) end
    // voxel: each DDA step below advances exactly one axis by one index, so this is
    // precisely the path length in voxels.
    const int64_t total_steps = std::abs(ix_end - ix) + std::abs(iy_end - iy) + std::abs(iz_end - iz);

    const int64_t margin_voxels =
      std::max<int64_t>(0, static_cast<int64_t>(std::ceil(cfg_.free_end_margin / r0)));
    // Never mark the endpoint voxel itself (voxels_to_mark <= total_steps always), and
    // stop margin_voxels short of it besides.
    const int64_t voxels_to_mark = std::max<int64_t>(0, total_steps - margin_voxels);
    if (voxels_to_mark == 0) return;

    const bool height_band = cfg_.free_min_height != 0.0 || cfg_.free_max_height != 0.0;
    const double origin_z = origin.z();

    auto stepOf = [](double d) { return d > 0.0 ? 1 : (d < 0.0 ? -1 : 0); };
    const int step_x = stepOf(dir.x());
    const int step_y = stepOf(dir.y());
    const int step_z = stepOf(dir.z());

    const double kInf = std::numeric_limits<double>::infinity();
    auto initTMax = [&](int64_t idx, double o, double d, int step) {
      if (step == 0) return kInf;  // ray never crosses a boundary on this axis.
      const double boundary = (step > 0) ? static_cast<double>(idx + 1) * r0 : static_cast<double>(idx) * r0;
      return (boundary - o) / d;
    };
    double t_max_x = initTMax(ix, origin.x(), dir.x(), step_x);
    double t_max_y = initTMax(iy, origin.y(), dir.y(), step_y);
    double t_max_z = initTMax(iz, origin.z(), dir.z(), step_z);

    const double t_delta_x = (step_x != 0) ? r0 / std::abs(dir.x()) : kInf;
    const double t_delta_y = (step_y != 0) ? r0 / std::abs(dir.y()) : kInf;
    const double t_delta_z = (step_z != 0) ? r0 / std::abs(dir.z()) : kInf;

    for (int64_t iter = 0; iter < voxels_to_mark; ++iter) {
      bool in_band = true;
      if (height_band) {
        const double centre_z = (static_cast<double>(iz) + 0.5) * r0 - origin_z;
        in_band = centre_z >= cfg_.free_min_height && centre_z <= cfg_.free_max_height;
      }
      if (in_band) {
        lvl0.free_voxels.insert(packVoxel(ix, iy, iz));
        if (lvl0.free_voxels.size() > cfg_.max_free_voxels) {
          // Over budget: abandon free space entirely for this build rather than leave
          // a partial, ad-hoc-truncated channel active. Marking nothing free is always
          // sound (see isFreeBound()), so this only loosens the bound.
          free_space_abandoned_ = true;
          lvl0.free_voxels.clear();
          return;
        }
      }

      // Advance whichever axis reaches its next voxel boundary first.
      if (t_max_x <= t_max_y && t_max_x <= t_max_z) {
        ix += step_x;
        t_max_x += t_delta_x;
      } else if (t_max_y <= t_max_z) {
        iy += step_y;
        t_max_y += t_delta_y;
      } else {
        iz += step_z;
        t_max_z += t_delta_z;
      }
    }
  }

  /**
   * @brief Build the coarse levels, apply dilation, and build the free-space /
   * distance-field channels.
   *
   * Must be called once after all points have been inserted.
   */
  void finalize()
  {
    if (levels_.empty() || levels_[0].voxels.empty()) return;

    // Derive each coarse level from the level-0 index set by arithmetic right
    // shift, which is exact floor division by 2^l for signed indices.
    levels_.resize(static_cast<std::size_t>(cfg_.num_levels));
    for (int l = 1; l < cfg_.num_levels; ++l) {
      auto & lvl = levels_[static_cast<std::size_t>(l)];
      lvl.resolution = cfg_.min_voxel_size * static_cast<double>(1 << l);
      lvl.inv_resolution = 1.0 / lvl.resolution;
      lvl.dilated = false;
      lvl.voxels.clear();
      lvl.voxels.reserve(levels_[0].voxels.size() >> std::min(l * 2, 20));
      for (int64_t key : levels_[0].voxels) {
        int64_t ix, iy, iz;
        unpackVoxel(key, ix, iy, iz);
        lvl.voxels.insert(packVoxel(ix >> l, iy >> l, iz >> l));
      }
    }

    // Dilate every non-leaf level. Level 0 stays exact: it is the leaf, where
    // the true score is what we want, and dilating it would be both wrong and
    // the most expensive level to inflate.
    for (int l = 1; l < cfg_.num_levels; ++l) {
      dilateLevel(levels_[static_cast<std::size_t>(l)], static_cast<std::size_t>(l));
    }

    buildFreeSpace();
    buildDistanceField();
  }

  /// @brief Number of levels currently built.
  /// @return Level count.
  int numLevels() const
  {
    return static_cast<int>(levels_.size());
  }

  /// @brief Access one level.
  /// @param l Level index, 0 is finest.
  /// @return Const reference to the level.
  const VoxelLevel & level(int l) const
  {
    return levels_[static_cast<std::size_t>(l)];
  }

  /// @brief Whether the pyramid holds any occupancy.
  /// @return True if empty.
  bool empty() const
  {
    return levels_.empty() || levels_[0].voxels.empty();
  }

  /// @brief Points dropped because their voxel index exceeded the packing range.
  /// @return Dropped point count.
  std::size_t outOfRangePoints() const
  {
    return out_of_range_points_;
  }

  /// @brief Whether dilation was skipped at a level due to the memory budget.
  /// @param l Level index.
  /// @return True if that level probes its neighbourhood on the fly.
  bool dilationSkipped(int l) const
  {
    return l < static_cast<int>(dilation_skipped_.size()) && dilation_skipped_[static_cast<std::size_t>(l)];
  }

  /// @brief Number of known-free voxels at a level.
  /// @param level Level index, 0 is finest.
  /// @return Size of that level's free set (0 if free space was not built or was abandoned).
  std::size_t freeVoxelCount(int level) const
  {
    return levels_[static_cast<std::size_t>(level)].free_voxels.size();
  }

  /// @brief Whether the free-space budget guard fired and free space was abandoned.
  /// @return True if `cfg_.max_free_voxels` was exceeded during insertRay().
  bool freeSpaceAbandoned() const
  {
    return free_space_abandoned_;
  }

  /// @brief Number of stored distance-field entries at a level.
  /// @param level Level index, 0 is finest.
  /// @return Size of that level's score grid (0 if DistanceField scoring was not
  ///   requested, or was abandoned -- see distanceFieldAbandoned()).
  std::size_t scoreVoxelCount(int level) const
  {
    return levels_[static_cast<std::size_t>(level)].scores.size();
  }

  /// @brief Whether the distance-field budget guard fired and the field was abandoned.
  /// @return True if `cfg_.max_score_voxels` was exceeded while building the field, in
  ///   which case this build fell back to Occupancy mode -- see effectiveScoreMode().
  bool distanceFieldAbandoned() const
  {
    return distance_field_abandoned_;
  }

  /// @brief The scoring mode this build actually ended up with.
  ///
  /// Equal to `config().score_mode` except when the DistanceField build overflowed
  /// `max_score_voxels` and had to fall back, in which case this returns
  /// `ScoreMode::Occupancy` while `config().score_mode` still reports the caller's
  /// original request. Callers that hand a `SearchConfig`/`ScoreMode` to
  /// `bnb_search.hpp` should read this rather than `config().score_mode`, so a search
  /// never scores against an empty distance field.
  /// @return The mode this pyramid was actually built with.
  ScoreMode effectiveScoreMode() const
  {
    return distance_field_abandoned_ ? ScoreMode::Occupancy : cfg_.score_mode;
  }

  /// @brief Approximate total heap footprint.
  /// @return Estimated bytes across all levels.
  std::size_t memoryBytes() const
  {
    std::size_t total = 0;
    for (const auto & l : levels_) total += l.memoryBytes();
    return total;
  }

  /// @brief Release all memory held by the pyramid.
  void clear()
  {
    levels_.clear();
    levels_.shrink_to_fit();
    dilation_skipped_.clear();
    out_of_range_points_ = 0;
    free_space_abandoned_ = false;
    distance_field_abandoned_ = false;
  }

  /**
   * @brief Release every buffer the pyramid owns and, on glibc, ask the allocator to
   * return the freed arena to the OS.
   *
   * `clear()` alone frees every `std::vector`/`unordered_set`/`VoxelScoreGrid` buffer
   * the pyramid owns, but glibc's allocator does not always hand large freed regions
   * back to the OS on its own -- RSS can stay high after the buffers are logically gone,
   * which is exactly the symptom the caller needs memory to recover from across a
   * relocalization lock. `malloc_trim(0)` asks it to. Guarded by `__GLIBC__`; a no-op on
   * any other allocator, where `clear()` is already the whole story.
   */
  void releaseMemory()
  {
    clear();
#ifdef __GLIBC__
    malloc_trim(0);
#endif
  }

  /// @brief Configuration the pyramid was built with.
  /// @return Const reference to the config.
  const Config & config() const
  {
    return cfg_;
  }

private:
  /**
   * @brief Dilate a level by its 26-neighbourhood, subject to the memory budget.
   *
   * The 26-neighbourhood is required rather than the 6-neighbourhood: child
   * nodes displace a query point diagonally in general, and a 6-connected
   * dilation is not a valid upper bound, which would let branch-and-bound prune
   * the true pose.
   *
   * @param lvl Level to dilate in place.
   * @param index Level index, for recording a skipped dilation.
   */
  void dilateLevel(VoxelLevel & lvl, std::size_t index)
  {
    VoxelSet out;
    out.reserve(lvl.voxels.size() * 4);
    for (int64_t key : lvl.voxels) {
      int64_t ix, iy, iz;
      unpackVoxel(key, ix, iy, iz);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            out.insert(packVoxel(ix + dx, iy + dy, iz + dz));
          }
        }
      }
      if (out.size() > cfg_.max_voxels_per_level) {
        // Over budget: abandon the dilated set and keep the exact one. hitBound()
        // will probe the 27-neighbourhood per query instead.
        if (index < dilation_skipped_.size()) dilation_skipped_[index] = true;
        lvl.dilated = false;
        return;
      }
    }
    lvl.voxels = std::move(out);
    lvl.dilated = true;
  }

  /**
   * @brief Turn the raw level-0 raycast result into the per-level free-space channel.
   *
   * No-op unless `cfg_.build_free_space`, and also a no-op under
   * `ScoreMode::DistanceField` (see Config::score_mode and insertRay() -- with no rays
   * ever recorded, `free_voxels[0]` is empty regardless, but skipping the rest of this
   * function outright avoids walking empty sets through every step below for nothing).
   * Called once at the end of finalize(), after occupied levels 1..L-1 are built and
   * dilated, so the occupancy-wins erasure below can see the final level-0 occupied set.
   *
   * Order matters and mirrors the module doc:
   *  1. Occupancy always wins over free: erase from `free_voxels[0]` every voxel that
   *     is occupied at level 0, or (when `cfg_.free_clear_near_occupied`) merely
   *     26-adjacent to one. This is what makes free space "confidently free" -- a ray
   *     that grazed past a wall should not zero-penalise a pose that puts the wall
   *     exactly there.
   *  2. Build, level by level, the "fully free" set: a level-l cell is fully free iff
   *     all 8 of its level-(l-1) children are. This is deliberately strict (not "most
   *     of its children") -- see erosion below for why.
   *  3. Erode every level >= 1 by its own 26-neighbourhood: keep a cell only if all 26
   *     neighbours are also fully free. This is the same soundness move as occupied
   *     dilation, inverted: dilation INFLATES occupied so a coarse hit is a valid upper
   *     bound on a descendant's hit; erosion SHRINKS free so a coarse "definitely free"
   *     reading is a valid upper bound on a descendant's absence of penalty. A child
   *     pose can land in any of a parent cell's 26 neighbours (the same displacement
   *     `bnbCellCentre()` bounds for occupancy), so only a cell whose entire
   *     neighbourhood -- not just itself -- is fully free can be certified free for
   *     EVERY child. Skipping this and using the fully-free set directly would let a
   *     coarse "free" reading be wrong about a child one cell over, which would let
   *     branch-and-bound zero-penalise a pose it should not -- the free-space mirror of
   *     the bug a non-dilated occupied set would cause. Level 0 needs no erosion: it is
   *     the exact leaf, matching how it is also the one level left undilated above.
   */
  void buildFreeSpace()
  {
    if (!cfg_.build_free_space || cfg_.score_mode == ScoreMode::DistanceField) return;
    if (free_space_abandoned_) {
      for (auto & lvl : levels_) {
        lvl.free_voxels.clear();
        lvl.has_free = false;
      }
      return;
    }

    VoxelSet & free0 = levels_[0].free_voxels;
    if (!free0.empty()) {
      const VoxelSet & occ0 = levels_[0].voxels;
      VoxelSet to_erase;
      for (int64_t key : free0) {
        int64_t ix, iy, iz;
        unpackVoxel(key, ix, iy, iz);
        bool remove = false;
        if (cfg_.free_clear_near_occupied) {
          for (int dx = -1; dx <= 1 && !remove; ++dx) {
            for (int dy = -1; dy <= 1 && !remove; ++dy) {
              for (int dz = -1; dz <= 1 && !remove; ++dz) {
                if (occ0.find(packVoxel(ix + dx, iy + dy, iz + dz)) != occ0.end()) remove = true;
              }
            }
          }
        } else {
          remove = occ0.find(key) != occ0.end();
        }
        if (remove) to_erase.insert(key);
      }
      for (int64_t key : to_erase) free0.erase(key);
    }

    // full[l]: the fully-free set at level l, for l >= 1. full[0] is free0 itself, used
    // directly below rather than copied into full[0] (which stays unused).
    std::vector<VoxelSet> full(static_cast<std::size_t>(cfg_.num_levels));
    for (int l = 1; l < cfg_.num_levels; ++l) {
      const VoxelSet & children = (l == 1) ? free0 : full[static_cast<std::size_t>(l - 1)];
      std::unordered_map<int64_t, int, VoxelHash> child_counts;
      child_counts.reserve(children.size());
      for (int64_t key : children) {
        int64_t ix, iy, iz;
        unpackVoxel(key, ix, iy, iz);
        ++child_counts[packVoxel(ix >> 1, iy >> 1, iz >> 1)];
      }
      VoxelSet & out = full[static_cast<std::size_t>(l)];
      out.reserve(child_counts.size());
      for (const auto & kv : child_counts) {
        if (kv.second == 8) out.insert(kv.first);
      }
    }

    for (int l = 1; l < cfg_.num_levels; ++l) {
      const VoxelSet & src = full[static_cast<std::size_t>(l)];
      VoxelSet & dst = levels_[static_cast<std::size_t>(l)].free_voxels;
      dst.clear();
      dst.reserve(src.size());
      for (int64_t key : src) {
        int64_t ix, iy, iz;
        unpackVoxel(key, ix, iy, iz);
        bool all_free = true;
        for (int dx = -1; dx <= 1 && all_free; ++dx) {
          for (int dy = -1; dy <= 1 && all_free; ++dy) {
            for (int dz = -1; dz <= 1 && all_free; ++dz) {
              if (src.find(packVoxel(ix + dx, iy + dy, iz + dz)) == src.end()) all_free = false;
            }
          }
        }
        if (all_free) dst.insert(key);
      }
    }

    for (int l = 0; l < cfg_.num_levels; ++l) {
      auto & lvl = levels_[static_cast<std::size_t>(l)];
      lvl.has_free = !lvl.free_voxels.empty();
    }
  }

  /**
   * @brief Build the level-0 distance field and its max-pooled, max-dilated coarse mirror.
   *
   * No-op unless `cfg_.score_mode == ScoreMode::DistanceField`. Called once at the end of
   * finalize(), after the occupied levels are built (level 0 exact, levels >= 1 dilated),
   * since the coarse pooling below walks the same level-0 -> level-l parent/child
   * relationship those levels were derived from, and the kernel radius is expressed in
   * level-0 voxels (Config::df_truncation_voxels).
   *
   * Construction, per CONTRACT_DF.md:
   *  1. Level 0: for every occupied level-0 voxel `v` and every offset `o` with
   *     `|o|_inf <= df_truncation_voxels` (a (2k+1)^3 cube, not a sphere -- computing the
   *     exact per-cell distance is unnecessary at this truncation, so the cube is the
   *     approximation the whole kernel-dilation construction rests on; see the module
   *     contract), `maxInsert(v + o, kernel[|o|_euclid])` where `kernel[m] =
   *     round(255 * exp(-(m*r0)^2 / (2*sigma^2)))`. This is precisely the same
   *     Cartographer-style max-dilation blur the occupied set's dilateLevel() performs,
   *     with `max` in place of set union and a radial falloff in place of a flat 1.
   *  2. Level l >= 1: MAX-POOL the 8 level-(l-1) children into their level-l parent, then
   *     MAX-DILATE the pooled result over its own 26-neighbourhood, and store that.
   *
   * Soundness: a query point's representative pose at a node moves by at most `r_l` per
   * axis between the node and any of its descendants (the same bnbCellCentre() /
   * hitBound() invariant the occupied dilation relies on), so the point stays inside the
   * stored level's 3x3x3 dilated block. The stored value there is a MAX over that block,
   * each cell of which is itself a max over everything below it, so it upper-bounds every
   * descendant's exact level-0 value -- the same argument as the occupancy dilation, with
   * `max` replacing set union, and simpler because there is no eroded free-space channel
   * to reason about in parallel (DistanceField ignores free space entirely; see
   * Config::score_mode).
   *
   * Budget: `cfg_.max_score_voxels` bounds the total stored entries across all levels. On
   * overflow the field is abandoned outright -- every level's `scores` grid is cleared --
   * and this build falls back to Occupancy mode (see distanceFieldAbandoned(),
   * effectiveScoreMode()) rather than serve a partially-built field, which would silently
   * under-score every point past whichever occupied voxel was being processed when the
   * budget tripped.
   */
  void buildDistanceField()
  {
    if (cfg_.score_mode != ScoreMode::DistanceField) return;
    if (levels_.empty() || levels_[0].voxels.empty()) return;

    const int k = std::max(0, cfg_.df_truncation_voxels);
    const double r0 = levels_[0].resolution;
    const double sigma = std::max(1e-9, cfg_.df_sigma);

    // Kernel indexed by squared voxel-offset magnitude (dx^2+dy^2+dz^2), so the (2k+1)^3
    // offsets visited per occupied voxel below share at most 3k^2+1 distinct exp()
    // evaluations instead of recomputing exp() per offset per occupied voxel.
    const int max_sq = 3 * k * k;
    std::vector<uint8_t> kernel(static_cast<std::size_t>(max_sq) + 1, 0);
    for (int sq = 0; sq <= max_sq; ++sq) {
      const double d = std::sqrt(static_cast<double>(sq)) * r0;
      kernel[static_cast<std::size_t>(sq)] =
        static_cast<uint8_t>(std::lround(255.0 * std::exp(-(d * d) / (2.0 * sigma * sigma))));
    }

    VoxelLevel & lvl0 = levels_[0];
    lvl0.scores.clear();
    bool overflow = false;
    for (int64_t key : lvl0.voxels) {
      int64_t ix, iy, iz;
      unpackVoxel(key, ix, iy, iz);
      for (int dx = -k; dx <= k; ++dx) {
        for (int dy = -k; dy <= k; ++dy) {
          for (int dz = -k; dz <= k; ++dz) {
            const int sq = dx * dx + dy * dy + dz * dz;
            const uint8_t val = kernel[static_cast<std::size_t>(sq)];
            // A stored zero is indistinguishable from absent (see VoxelScoreGrid::at()),
            // so skip it: it costs a slot for no scoring benefit.
            if (val == 0) continue;
            lvl0.scores.maxInsert(packVoxel(ix + dx, iy + dy, iz + dz), val);
          }
        }
      }
      if (lvl0.scores.size() > cfg_.max_score_voxels) {
        overflow = true;
        break;
      }
    }

    if (!overflow) {
      // Coarse levels: max-pool each level's 8 children into their parent, then
      // max-dilate the pooled result over the 26-neighbourhood -- see the soundness
      // argument above this function.
      for (int l = 1; l < cfg_.num_levels && !overflow; ++l) {
        VoxelLevel & lvl = levels_[static_cast<std::size_t>(l)];
        VoxelLevel & child = levels_[static_cast<std::size_t>(l - 1)];

        VoxelScoreGrid pooled;
        child.scores.forEach([&](int64_t child_key, uint8_t val) {
          int64_t cx, cy, cz;
          unpackVoxel(child_key, cx, cy, cz);
          pooled.maxInsert(packVoxel(cx >> 1, cy >> 1, cz >> 1), val);
        });

        lvl.scores.clear();
        pooled.forEach([&](int64_t pooled_key, uint8_t val) {
          int64_t px, py, pz;
          unpackVoxel(pooled_key, px, py, pz);
          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dz = -1; dz <= 1; ++dz) {
                lvl.scores.maxInsert(packVoxel(px + dx, py + dy, pz + dz), val);
              }
            }
          }
        });

        if (pooled.size() > cfg_.max_score_voxels || lvl.scores.size() > cfg_.max_score_voxels) {
          overflow = true;
        }
      }
    }

    if (overflow) {
      // Over budget: abandon the field entirely rather than serve a partially-built one,
      // and fall back to Occupancy mode for this build (see effectiveScoreMode()).
      for (auto & lvl : levels_) lvl.scores.clear();
      distance_field_abandoned_ = true;
    }
  }

  Config cfg_;
  std::vector<VoxelLevel> levels_;
  std::vector<bool> dilation_skipped_;
  std::size_t out_of_range_points_ = 0;
  bool free_space_abandoned_ = false;
  bool distance_field_abandoned_ = false;
};

}  // namespace eidos::reloc
