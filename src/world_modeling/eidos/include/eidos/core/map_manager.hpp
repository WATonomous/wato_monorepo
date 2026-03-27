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

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <any>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "eidos/utils/types.hpp"

// Forward declare sqlite3
struct sqlite3;

namespace eidos
{

/**
 * @brief Centralized keyframe pose + data store with SQLite persistence.
 *
 * At runtime, all data lives in memory as std::any (unchanged from v1).
 * Persistence uses a single SQLite .map file. Plugins register their
 * data formats via registerKeyframeFormat/registerGlobalFormat — the
 * shared format registry handles serialization.
 *
 * The .map file is self-describing: a data_formats table stores which
 * format each data_key uses, so any reader (eidos_tools CLI) can
 * deserialize without external knowledge.
 */
class MapManager
{
public:
  MapManager();
  ~MapManager();

  // ---- Format registration ----

  /**
   * @brief Register a keyframe data format for persistence.
   *
   * Type contract: the std::any stored via store() for this data_key
   * must contain the C++ type that the named format expects. E.g. "pcl_pcd_binary"
   * expects pcl::PointCloud<PointType>::Ptr. A mismatch causes bad_any_cast at save time.
   *
   * @param data_key  Key used in store (e.g. "liso_factor/cloud").
   * @param format    Format name from the shared registry (e.g. "pcl_pcd_binary").
   */
  void registerKeyframeFormat(const std::string & data_key, const std::string & format);

  /**
   * @brief Register a global data format for persistence. Same type contract.
   */
  void registerGlobalFormat(const std::string & data_key, const std::string & format);

  // ---- Keyframe pose management ----

  void addKeyframe(gtsam::Key gtsam_key, const PoseType & pose, const std::string & owner = "");
  std::string getOwnerPlugin(gtsam::Key gtsam_key) const;
  void updatePoses(const gtsam::Values & optimized);

  pcl::PointCloud<PointType>::Ptr getKeyPoses3D() const;
  pcl::PointCloud<PoseType>::Ptr getKeyPoses6D() const;
  pcl::KdTreeFLANN<PointType>::Ptr getKdTree();
  int numKeyframes() const;
  std::vector<gtsam::Key> getKeyList() const;
  int getCloudIndex(gtsam::Key gtsam_key) const;
  gtsam::Key getKeyFromCloudIndex(int cloud_index) const;

  // ---- Typed data storage (runtime, in-memory) ----
  // std::any is internal — plugins use store<T>() / retrieve<T>().

  /// Store typed keyframe data. T is wrapped in std::any internally.
  template <typename T>
  void store(gtsam::Key key, const std::string & data_key, T value)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    keyframe_data_[key][data_key] = std::any(std::move(value));
  }

  /// Retrieve typed keyframe data. Returns std::nullopt if key/data_key missing or type mismatch.
  template <typename T>
  std::optional<T> retrieve(gtsam::Key key, const std::string & data_key) const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    auto kf = keyframe_data_.find(key);
    if (kf == keyframe_data_.end()) return std::nullopt;
    auto it = kf->second.find(data_key);
    if (it == kf->second.end()) return std::nullopt;
    try {
      return std::any_cast<T>(it->second);
    } catch (...) {
      return std::nullopt;
    }
  }

  /// Store typed global data.
  template <typename T>
  void storeGlobal(const std::string & data_key, T value)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    global_data_[data_key] = std::any(std::move(value));
  }

  /// Retrieve typed global data.
  template <typename T>
  std::optional<T> retrieveGlobal(const std::string & data_key) const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = global_data_.find(data_key);
    if (it == global_data_.end()) return std::nullopt;
    try {
      return std::any_cast<T>(it->second);
    } catch (...) {
      return std::nullopt;
    }
  }

  /// Check if keyframe data exists for a given key + data_key.
  bool hasKeyframeData(gtsam::Key key, const std::string & data_key) const;

  // ---- Graph adjacency ----

  void addEdges(const gtsam::NonlinearFactorGraph & new_factors, const std::vector<std::string> & factor_owners = {});
  std::unordered_map<gtsam::Key, std::vector<gtsam::Key>> getAdjacency() const;
  std::string getEdgeOwner(gtsam::Key key_a, gtsam::Key key_b) const;

  // ---- Persistence (single .map SQLite file) ----

  /// Save everything to a .map file. Creates/overwrites the file.
  bool saveMap(const std::string & path);

  /// Load from a .map file. Poses loaded immediately, blobs on demand.
  bool loadMap(const std::string & path);

  bool hasPriorMap() const;
  bool isPriorMapKey(gtsam::Key key) const;

private:
  pcl::PointCloud<PointType>::Ptr key_poses_3d_;
  pcl::PointCloud<PoseType>::Ptr key_poses_6d_;

  std::map<gtsam::Key, int> key_to_cloud_index_;
  std::vector<gtsam::Key> key_list_;
  std::map<gtsam::Key, std::string> key_owner_plugin_;

  std::map<gtsam::Key, std::unordered_map<std::string, std::any>> keyframe_data_;
  std::unordered_map<std::string, std::any> global_data_;
  std::unordered_map<gtsam::Key, std::vector<gtsam::Key>> adjacency_;
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::string> edge_owners_;

  pcl::KdTreeFLANN<PointType>::Ptr kdtree_;
  bool kdtree_dirty_ = true;

  std::set<gtsam::Key> prior_map_keys_;
  bool prior_map_loaded_ = false;

  // ---- Format registration ----
  std::unordered_map<std::string, std::string> keyframe_formats_;  ///< data_key → format name
  std::unordered_map<std::string, std::string> global_formats_;  ///< data_key → format name

  // ---- SQLite handle for lazy loading ----
  sqlite3 * load_db_ = nullptr;

  mutable std::mutex mtx_;
};

}  // namespace eidos
