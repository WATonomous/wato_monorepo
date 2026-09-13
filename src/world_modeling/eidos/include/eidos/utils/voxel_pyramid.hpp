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
#include <climits>
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
  #include <malloc.h>  // malloc_trim() in releaseMemory()
#endif

#include <omp.h>  // buildDistanceField()'s parallel splat

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eidos::reloc
{

// 21 bits/axis -> +/-1048km at 1m resolution, far beyond any plausible map extent.
constexpr int kVoxelBits = 21;
constexpr int64_t kVoxelBias = 1LL << (kVoxelBits - 1);
constexpr int64_t kVoxelMask = (1LL << kVoxelBits) - 1;
constexpr int64_t kVoxelIndexLimit = kVoxelBias - 1;

// Packs signed voxel indices into one 64-bit key (ix low, iy mid, iz high bits).
inline int64_t packVoxel(int64_t ix, int64_t iy, int64_t iz)
{
  return ((ix + kVoxelBias) & kVoxelMask) | (((iy + kVoxelBias) & kVoxelMask) << kVoxelBits) |
         (((iz + kVoxelBias) & kVoxelMask) << (2 * kVoxelBits));
}

inline void unpackVoxel(int64_t key, int64_t & ix, int64_t & iy, int64_t & iz)
{
  ix = (key & kVoxelMask) - kVoxelBias;
  iy = ((key >> kVoxelBits) & kVoxelMask) - kVoxelBias;
  iz = ((key >> (2 * kVoxelBits)) & kVoxelMask) - kVoxelBias;
}

// floor(), not a cast: casts truncate toward zero and misfile negative coordinates.
inline int64_t voxelIndex(double v, double inv_resolution)
{
  return static_cast<int64_t>(std::floor(v * inv_resolution));
}

// splitmix64 finalizer.
struct VoxelHash
{
  std::size_t operator()(int64_t k) const noexcept
  {
    uint64_t x = static_cast<uint64_t>(k) + 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return static_cast<std::size_t>(x ^ (x >> 31));
  }
};

using VoxelSet = std::unordered_set<int64_t, VoxelHash>;

// Occupancy = ternary hit/free/unknown score. DistanceField (default) = continuous falloff
// around the nearest occupied voxel; binary containment carries almost no alignment signal on
// a vegetated scene (see CONTRACT_DF.md).
enum class ScoreMode
{
  Occupancy,
  DistanceField
};

// Open-addressed voxel->score map, backed by two flat vectors instead of unordered_set nodes:
// freeing millions of unordered_set nodes doesn't return memory to the OS, and RSS needs to
// actually drop after a relocalization lock. Flat probing is also faster in the scoring hot path.
class VoxelScoreGrid
{
public:
  static constexpr int64_t kEmptyKey = INT64_MIN;  // never a legal packed key (see packVoxel())
  static constexpr double kMaxLoadFactor = 0.6;

  void reserve(std::size_t expected_elements)
  {
    if (expected_elements == 0) return;
    const std::size_t needed =
      static_cast<std::size_t>(std::ceil(static_cast<double>(expected_elements) / kMaxLoadFactor));
    const std::size_t target = roundUpPow2(std::max<std::size_t>(needed, kInitialCapacity));
    if (target > capacity_) rehash(target);
  }

  // grid[key] = max(grid[key], score).
  void maxInsert(int64_t key, uint8_t score)
  {
    if (capacity_ == 0) rehash(kInitialCapacity);
    const std::size_t idx = slotFor(key);
    if (keys_[idx] == kEmptyKey) {
      keys_[idx] = key;
      values_[idx] = score;
      ++size_;
      if (static_cast<double>(size_) > kMaxLoadFactor * static_cast<double>(capacity_)) {
        rehash(capacity_ * 2);
      }
    } else if (score > values_[idx]) {
      values_[idx] = score;
    }
  }

  // Absent == 0, indistinguishable from a stored zero.
  uint8_t at(int64_t key) const
  {
    if (capacity_ == 0) return 0;
    const std::size_t idx = slotFor(key);
    return keys_[idx] == kEmptyKey ? static_cast<uint8_t>(0) : values_[idx];
  }

  // Warms the cache line for `key`; purely advisory (see scorePoseAtLevel()'s pipelining).
  void prefetch(int64_t key) const
  {
    if (capacity_ == 0) return;
    __builtin_prefetch(&keys_[VoxelHash{}(key) & (capacity_ - 1)], 0, 1);
  }

  std::size_t size() const
  {
    return size_;
  }

  std::size_t capacity() const
  {
    return capacity_;
  }

  std::size_t memoryBytes() const
  {
    return capacity_ * (sizeof(int64_t) + sizeof(uint8_t));
  }

  bool empty() const
  {
    return size_ == 0;
  }

  // Swap-with-empty, not vector::clear(): clear() keeps the buffer allocated, defeating the
  // point of this class over a VoxelSet.
  void clear()
  {
    std::vector<int64_t>().swap(keys_);
    std::vector<uint8_t>().swap(values_);
    capacity_ = 0;
    size_ = 0;
  }

  template <class F>
  void forEach(F && f) const
  {
    for (std::size_t i = 0; i < capacity_; ++i) {
      if (keys_[i] != kEmptyKey) f(keys_[i], values_[i]);
    }
  }

private:
  static constexpr std::size_t kInitialCapacity = 16;

  static std::size_t roundUpPow2(std::size_t v)
  {
    std::size_t p = 1;
    while (p < v) p <<= 1;
    return p;
  }

  std::size_t slotFor(int64_t key) const
  {
    const std::size_t mask = capacity_ - 1;
    std::size_t idx = VoxelHash{}(key)&mask;
    while (keys_[idx] != kEmptyKey && keys_[idx] != key) {
      idx = (idx + 1) & mask;
    }
    return idx;
  }

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

// One resolution level. Non-leaf levels are normally pre-dilated by their 26-neighbourhood so a
// lookup is a valid branch-and-bound upper bound; if dilation would exceed budget, the exact set
// is kept and hitBound() probes the 27-neighbourhood on the fly instead.
struct VoxelLevel
{
  double resolution = 1.0;
  double inv_resolution = 1.0;
  VoxelSet voxels;  // Occupied set, dilated when `dilated`.
  bool dilated = false;
  VoxelSet free_voxels;  // Known-empty. Level 0 exact; level>0 eroded (see isFreeBound()).
  bool has_free = false;
  VoxelScoreGrid scores;  // Distance-field values; empty in Occupancy mode.

  bool hit(const Eigen::Vector3d & p) const
  {
    return voxels.find(packVoxel(
             voxelIndex(p.x(), inv_resolution),
             voxelIndex(p.y(), inv_resolution),
             voxelIndex(p.z(), inv_resolution))) != voxels.end();
  }

  // 26-neighbourhood occupancy bound: one probe if dilated, else 27 probes.
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

  bool isFree(const Eigen::Vector3d & p) const
  {
    return free_voxels.find(packVoxel(
             voxelIndex(p.x(), inv_resolution),
             voxelIndex(p.y(), inv_resolution),
             voxelIndex(p.z(), inv_resolution))) != free_voxels.end();
  }

  // True only if free at level 0 for EVERY pose a descendant node can take (eroded by
  // finalize(), the free-space mirror of occupied dilation).
  bool isFreeBound(const Eigen::Vector3d & p) const
  {
    if (!has_free) return false;
    return isFree(p);
  }

  uint8_t scoreAt(const Eigen::Vector3d & p) const
  {
    return scores.at(scoreKey(p));
  }

  // Split from scoreAt() so a caller can prefetch ahead of scoring.
  int64_t scoreKey(const Eigen::Vector3d & p) const
  {
    return packVoxel(
      voxelIndex(p.x(), inv_resolution), voxelIndex(p.y(), inv_resolution), voxelIndex(p.z(), inv_resolution));
  }

  uint8_t scoreAtKey(int64_t key) const
  {
    return scores.at(key);
  }

  // Levels > 0 are pre-max-pooled/dilated by buildDistanceField(), so one lookup is already a
  // valid upper bound -- unlike hitBound() there's no on-the-fly fallback needed.
  uint8_t scoreBound(const Eigen::Vector3d & p) const
  {
    return scoreAt(p);
  }

  // Estimate, not exact (unordered_set node/bucket accounting); free_voxels counted too since
  // it's a volume that can rival the occupied surface.
  std::size_t memoryBytes() const
  {
    const auto set_bytes = [](const VoxelSet & s) {
      return s.size() * (sizeof(int64_t) + sizeof(void *)) + s.bucket_count() * sizeof(void *);
    };
    return set_bytes(voxels) + set_bytes(free_voxels) + scores.memoryBytes();
  }
};

// Multi-resolution sparse occupancy pyramid over a prior point cloud map. Level 0 (finest) is
// kept exact/undilated as the branch-and-bound leaf; coarser levels double in resolution and are
// derived from the level-0 index set by bit-shift (exact, since resolutions are power-of-two
// multiples anchored at the origin, and far cheaper than rebuilding from raw points).
class VoxelPyramid
{
public:
  struct Config
  {
    double min_voxel_size = 1.0;
    int num_levels = 6;
    double max_height = 6.0;  // Discards points above this map-frame z; <= 0 disables.
    std::size_t max_voxels_per_level = 8000000;  // Dilation budget per level.

    bool build_free_space = false;  // false => byte-identical to the pre-free-space pyramid.
    double free_max_range = 40.0;
    double free_end_margin = 1.0;  // Stop short of the ray endpoint; the surface itself isn't free.
    double free_min_height = 0.0;  // Band on (voxel z - ray origin z); both 0 disables it.
    double free_max_height = 0.0;
    bool free_clear_near_occupied = true;  // Also clear free voxels 26-adjacent to occupied
      // ones, so registration jitter isn't penalised.
    std::size_t max_free_voxels = 20000000;  // On overflow, abandon free space (always sound).

    ScoreMode score_mode = ScoreMode::DistanceField;  // Under DistanceField, insertRay()/
      // buildFreeSpace() are no-ops -- the falloff already
      // covers what the ternary free state approximated.
    double df_sigma = 1.0;  // Falloff std-dev (m); ~registration offset to absorb.
    int df_truncation_voxels = 2;  // Kernel radius in level-0 voxels; beyond it, score 0.
    std::size_t max_score_voxels = 40000000;  // On overflow, abandon the field and fall back to
      // Occupancy (see distanceFieldAbandoned()).
    int build_threads = 1;  // Threads for buildDistanceField()'s level-0 splat (the
      // most expensive finalize() step). 1 = serial.
  };

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

  // Drops points above max_height and out-of-range voxel indices (counted, not aliased).
  void insert(const Eigen::Vector3d & p)
  {
    if (cfg_.max_height > 0.0 && p.z() > cfg_.max_height) return;
    const double inv = levels_[0].inv_resolution;
    const int64_t ix = voxelIndex(p.x(), inv);
    const int64_t iy = voxelIndex(p.y(), inv);
    const int64_t iz = voxelIndex(p.z(), inv);
    if (std::abs(ix) > kVoxelIndexLimit || std::abs(iy) > kVoxelIndexLimit || std::abs(iz) > kVoxelIndexLimit) {
      ++out_of_range_points_;
      return;
    }
    levels_[0].voxels.insert(packVoxel(ix, iy, iz));
  }

  // Thread-safe insert(): writes into a caller-owned shard instead of the shared level-0 set, so
  // any number of threads can call this concurrently (each with its own shard). Merge with
  // mergeShardsIntoLevel0() once done -- union is commutative, so the result is bit-identical to
  // serial insert() in any order. No free-space equivalent: insertRay() mutates shared state
  // (free_space_abandoned_, the free set) this can't safely touch.
  bool insertIntoShard(const Eigen::Vector3d & p, VoxelSet & shard) const
  {
    if (cfg_.max_height > 0.0 && p.z() > cfg_.max_height) return true;
    const double inv = levels_[0].inv_resolution;
    const int64_t ix = voxelIndex(p.x(), inv);
    const int64_t iy = voxelIndex(p.y(), inv);
    const int64_t iz = voxelIndex(p.z(), inv);
    if (std::abs(ix) > kVoxelIndexLimit || std::abs(iy) > kVoxelIndexLimit || std::abs(iz) > kVoxelIndexLimit) {
      return false;
    }
    shard.insert(packVoxel(ix, iy, iz));
    return true;
  }

  // Unions per-thread shards into level 0. Call once, single-threaded, between beginInsert() and
  // finalize(). Shards are drained (left empty) rather than copied.
  void mergeShardsIntoLevel0(std::vector<VoxelSet> & shards, std::size_t out_of_range_points)
  {
    VoxelSet & dst = levels_[0].voxels;
    std::size_t total = 0;
    for (const auto & s : shards) total += s.size();
    dst.reserve(dst.size() + total);
    for (auto & s : shards) {
      for (int64_t key : s) dst.insert(key);
      VoxelSet().swap(s);
    }
    out_of_range_points_ += out_of_range_points;
  }

  // Marks voxels along origin->endpoint as known-free at level 0, via Amanatides-Woo DDA
  // traversal. No-op unless build_free_space (or under DistanceField, or once free space is
  // already abandoned). Not thread-safe (same contract as insert()).
  void insertRay(const Eigen::Vector3d & origin, const Eigen::Vector3d & endpoint)
  {
    if (!cfg_.build_free_space || free_space_abandoned_ || cfg_.score_mode == ScoreMode::DistanceField) return;

    const Eigen::Vector3d delta = endpoint - origin;
    const double length = delta.norm();
    constexpr double kMinRayLength = 1e-6;
    if (length < kMinRayLength) return;

    const double clamped_length = std::min(length, cfg_.free_max_range);
    const Eigen::Vector3d dir = delta / length;
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

    const int64_t total_steps = std::abs(ix_end - ix) + std::abs(iy_end - iy) + std::abs(iz_end - iz);

    const int64_t margin_voxels = std::max<int64_t>(0, static_cast<int64_t>(std::ceil(cfg_.free_end_margin / r0)));
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
      if (step == 0) return kInf;
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
          free_space_abandoned_ = true;
          lvl0.free_voxels.clear();
          return;
        }
      }

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

  // Builds coarse levels, dilates them, and builds the free-space/distance-field channels. Call
  // once after all points are inserted.
  void finalize()
  {
    if (levels_.empty() || levels_[0].voxels.empty()) return;

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

    // Level 0 stays exact: it's the leaf, where the true score is what's wanted.
    for (int l = 1; l < cfg_.num_levels; ++l) {
      dilateLevel(levels_[static_cast<std::size_t>(l)], static_cast<std::size_t>(l));
    }

    buildFreeSpace();
    buildDistanceField();
  }

  int numLevels() const
  {
    return static_cast<int>(levels_.size());
  }

  const VoxelLevel & level(int l) const
  {
    return levels_[static_cast<std::size_t>(l)];
  }

  bool empty() const
  {
    return levels_.empty() || levels_[0].voxels.empty();
  }

  std::size_t outOfRangePoints() const
  {
    return out_of_range_points_;
  }

  bool dilationSkipped(int l) const
  {
    return l < static_cast<int>(dilation_skipped_.size()) && dilation_skipped_[static_cast<std::size_t>(l)];
  }

  std::size_t freeVoxelCount(int level) const
  {
    return levels_[static_cast<std::size_t>(level)].free_voxels.size();
  }

  bool freeSpaceAbandoned() const
  {
    return free_space_abandoned_;
  }

  std::size_t scoreVoxelCount(int level) const
  {
    return levels_[static_cast<std::size_t>(level)].scores.size();
  }

  bool distanceFieldAbandoned() const
  {
    return distance_field_abandoned_;
  }

  // Actual mode this build ended up with -- differs from config().score_mode only after a
  // DistanceField overflow fallback. Callers should read this, not config().score_mode.
  ScoreMode effectiveScoreMode() const
  {
    return distance_field_abandoned_ ? ScoreMode::Occupancy : cfg_.score_mode;
  }

  std::size_t memoryBytes() const
  {
    std::size_t total = 0;
    for (const auto & l : levels_) total += l.memoryBytes();
    return total;
  }

  void clear()
  {
    levels_.clear();
    levels_.shrink_to_fit();
    dilation_skipped_.clear();
    out_of_range_points_ = 0;
    free_space_abandoned_ = false;
    distance_field_abandoned_ = false;
  }

  // clear() plus malloc_trim(): glibc doesn't always hand freed arenas back to the OS on its
  // own, and RSS needs to actually drop after a relocalization lock.
  void releaseMemory()
  {
    clear();
#ifdef __GLIBC__
    malloc_trim(0);
#endif
  }

  const Config & config() const
  {
    return cfg_;
  }

private:
  // 26- not 6-neighbourhood: children displace diagonally, so 6-connected isn't a valid bound.
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
        // Over budget: keep the exact set; hitBound() probes on the fly instead.
        if (index < dilation_skipped_.size()) dilation_skipped_[index] = true;
        lvl.dilated = false;
        return;
      }
    }
    lvl.voxels = std::move(out);
    lvl.dilated = true;
  }

  // Turns the level-0 raycast result into the per-level free-space channel: occupancy always
  // wins (erase free voxels at/near occupied ones), then build+erode each coarser level's
  // "fully free" set so a coarse reading stays a sound upper bound for every descendant -- the
  // free-space mirror of occupied dilation (see buildDistanceField()'s soundness note).
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

  // Builds the level-0 distance field (max-dilate a falloff kernel around each occupied voxel)
  // and its max-pooled, max-dilated coarse mirror -- same construction as dilateLevel(), with
  // max replacing set union, so the same soundness argument applies: a node's descendants can't
  // move a query point outside the stored level's dilated block, so the stored max upper-bounds
  // every descendant's exact value. No-op outside DistanceField mode. On overflow
  // (max_score_voxels), abandons the whole field and falls back to Occupancy rather than serve a
  // partially-built one.
  void buildDistanceField()
  {
    if (cfg_.score_mode != ScoreMode::DistanceField) return;
    if (levels_.empty() || levels_[0].voxels.empty()) return;

    const int k = std::max(0, cfg_.df_truncation_voxels);
    const double r0 = levels_[0].resolution;
    const double sigma = std::max(1e-9, cfg_.df_sigma);

    // Kernel indexed by squared offset so the (2k+1)^3 cube shares exp() evaluations.
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
    const int nthreads = std::max(1, cfg_.build_threads);
    if (nthreads > 1 && lvl0.voxels.size() > 4096) {
      // Sort by packed key (= spatial order: iz high bits, iy mid, ix low) before splitting
      // statically across threads, so each thread's slice is a spatially compact region and
      // shards barely overlap. An earlier unsorted/dynamic split let spatially-adjacent voxels
      // land in different threads' shards, so every shard ended up splatting nearly the whole
      // grid and the merge below was slower than the serial loop it replaced.
      std::vector<int64_t> occ(lvl0.voxels.begin(), lvl0.voxels.end());
      std::sort(occ.begin(), occ.end());
      std::vector<VoxelScoreGrid> shards(static_cast<std::size_t>(nthreads));

#pragma omp parallel for schedule(static) num_threads(nthreads)
      for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(occ.size()); ++i) {
        int64_t ix, iy, iz;
        unpackVoxel(occ[static_cast<std::size_t>(i)], ix, iy, iz);
        VoxelScoreGrid & shard = shards[static_cast<std::size_t>(omp_get_thread_num())];
        for (int dx = -k; dx <= k; ++dx) {
          for (int dy = -k; dy <= k; ++dy) {
            for (int dz = -k; dz <= k; ++dz) {
              const int sq = dx * dx + dy * dy + dz * dz;
              const uint8_t val = kernel[static_cast<std::size_t>(sq)];
              if (val == 0) continue;
              shard.maxInsert(packVoxel(ix + dx, iy + dy, iz + dz), val);
            }
          }
        }
      }

      // max is commutative/associative, so this reproduces the serial loop's result exactly.
      for (auto & shard : shards) {
        shard.forEach([&](int64_t k2, uint8_t v2) { lvl0.scores.maxInsert(k2, v2); });
      }
      overflow = lvl0.scores.size() > cfg_.max_score_voxels;
    } else {
      for (int64_t key : lvl0.voxels) {
        int64_t ix, iy, iz;
        unpackVoxel(key, ix, iy, iz);
        for (int dx = -k; dx <= k; ++dx) {
          for (int dy = -k; dy <= k; ++dy) {
            for (int dz = -k; dz <= k; ++dz) {
              const int sq = dx * dx + dy * dy + dz * dz;
              const uint8_t val = kernel[static_cast<std::size_t>(sq)];
              if (val == 0) continue;  // absent == 0, so skip storing it
              lvl0.scores.maxInsert(packVoxel(ix + dx, iy + dy, iz + dz), val);
            }
          }
        }
        if (lvl0.scores.size() > cfg_.max_score_voxels) {
          overflow = true;
          break;
        }
      }
    }

    if (!overflow) {
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
