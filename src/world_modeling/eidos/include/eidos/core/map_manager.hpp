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
  /// @brief Construct the MapManager with empty keyframe and data stores.
  MapManager();

  /// @brief Destructor. Closes any open SQLite handle used for lazy loading.
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
   * @brief Register a global data format for persistence.
   *
   * Same type contract as registerKeyframeFormat: the std::any stored via
   * storeGlobal() for this data_key must match the C++ type expected by the
   * named format.
   * @param data_key Key used in storeGlobal (e.g. "global_map/cloud").
   * @param format   Format name from the shared registry.
   */
  void registerGlobalFormat(const std::string & data_key, const std::string & format);

  // ---- Keyframe pose management ----

  /**
   * @brief Add a new keyframe with its 6-DOF pose.
   * @param gtsam_key The GTSAM key identifying this keyframe.
   * @param pose      The 6-DOF pose (PointXYZIRPYT) for the keyframe.
   * @param owner     Name of the plugin that created this keyframe (empty if unowned).
   */
  void addKeyframe(gtsam::Key gtsam_key, const PoseType & pose, const std::string & owner = "");

  /**
   * @brief Get the name of the plugin that created a given keyframe.
   * @param gtsam_key The GTSAM key to look up.
   * @return The owner plugin name, or an empty string if not found.
   */
  std::string getOwnerPlugin(gtsam::Key gtsam_key) const;

  /**
   * @brief Update all stored keyframe poses from freshly optimized values.
   *
   * Iterates over all known keys and updates the 3D and 6D point clouds
   * with the optimized Pose3 values.
   * @param optimized The full set of optimized GTSAM Values.
   */
  void updatePoses(const gtsam::Values & optimized);

  /// @brief Get the 3D keyframe positions (XYZ only).
  /// @return Shared pointer to the 3D key poses point cloud.
  pcl::PointCloud<PointType>::Ptr getKeyPoses3D() const;

  /// @brief Get the 6-DOF keyframe poses (XYZ + intensity + RPY + time).
  /// @return Shared pointer to the 6D key poses point cloud.
  pcl::PointCloud<PoseType>::Ptr getKeyPoses6D() const;

  /**
   * @brief Get a KD-tree built over the 3D keyframe positions.
   *
   * Lazily rebuilds the KD-tree when the poses have been modified since
   * the last call.
   * @return Shared pointer to the KD-tree.
   */
  pcl::KdTreeFLANN<PointType>::Ptr getKdTree();

  /// @brief Get the total number of keyframes stored.
  /// @return Number of keyframes.
  int numKeyframes() const;

  /// @brief Get an ordered list of all GTSAM keys in insertion order.
  /// @return Vector of GTSAM keys.
  std::vector<gtsam::Key> getKeyList() const;

  /**
   * @brief Map a GTSAM key to its index in the point cloud arrays.
   * @param gtsam_key The GTSAM key to look up.
   * @return The zero-based cloud index.
   */
  int getCloudIndex(gtsam::Key gtsam_key) const;

  /**
   * @brief Map a point cloud index back to its GTSAM key.
   * @param cloud_index The zero-based index in the point cloud.
   * @return The corresponding GTSAM key.
   */
  gtsam::Key getKeyFromCloudIndex(int cloud_index) const;

  // ---- Typed data storage (runtime, in-memory) ----
  // std::any is internal — plugins use store<T>() / retrieve<T>().

  /**
   * @brief Store typed keyframe data. T is wrapped in std::any internally.
   * @tparam T The data type to store.
   * @param key      The GTSAM key identifying the keyframe.
   * @param data_key A string key identifying the data slot (e.g. "liso_factor/cloud").
   * @param value    The value to store (moved into internal storage).
   * @note Thread-safe: acquires internal mutex.
   */
  template <typename T>
  void store(gtsam::Key key, const std::string & data_key, T value)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    keyframe_data_[key][data_key] = std::any(std::move(value));
  }

  /**
   * @brief Retrieve typed keyframe data.
   * @tparam T The expected data type.
   * @param key      The GTSAM key identifying the keyframe.
   * @param data_key A string key identifying the data slot.
   * @return The stored value, or std::nullopt if the key/data_key is missing
   *         or the stored type does not match T.
   * @note Thread-safe: acquires internal mutex.
   */
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

  /**
   * @brief Store typed global (non-keyframe) data.
   * @tparam T The data type to store.
   * @param data_key A string key identifying the data slot.
   * @param value    The value to store (moved into internal storage).
   * @note Thread-safe: acquires internal mutex.
   */
  template <typename T>
  void storeGlobal(const std::string & data_key, T value)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    global_data_[data_key] = std::any(std::move(value));
  }

  /**
   * @brief Retrieve typed global (non-keyframe) data.
   * @tparam T The expected data type.
   * @param data_key A string key identifying the data slot.
   * @return The stored value, or std::nullopt if the key is missing or the
   *         stored type does not match T.
   * @note Thread-safe: acquires internal mutex.
   */
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

  /**
   * @brief Check if keyframe data exists for a given key and data slot.
   * @param key      The GTSAM key identifying the keyframe.
   * @param data_key A string key identifying the data slot.
   * @return True if data is stored for that key/data_key pair.
   */
  bool hasKeyframeData(gtsam::Key key, const std::string & data_key) const;

  // ---- Graph adjacency ----

  /**
   * @brief Extract pairwise edges from new factors and add them to the
   *        adjacency map.
   * @param new_factors   Factors to scan for binary edges.
   * @param factor_owners Parallel vector of plugin names that own each factor
   *                      (may be empty).
   */
  void addEdges(const gtsam::NonlinearFactorGraph & new_factors, const std::vector<std::string> & factor_owners = {});

  /**
   * @brief Get the full adjacency map (key -> list of connected keys).
   * @return A copy of the adjacency map.
   */
  std::unordered_map<gtsam::Key, std::vector<gtsam::Key>> getAdjacency() const;

  /**
   * @brief Get the plugin name that owns the edge between two keys.
   * @param key_a First key of the edge.
   * @param key_b Second key of the edge.
   * @return The owner plugin name, or an empty string if not found.
   */
  std::string getEdgeOwner(gtsam::Key key_a, gtsam::Key key_b) const;

  // ---- Persistence (single .map SQLite file) ----

  /**
   * @brief Save all keyframes, data, and adjacency to a SQLite .map file.
   *
   * Creates or overwrites the file at the given path. Uses registered
   * format serializers for keyframe and global data blobs.
   * @param path Filesystem path for the .map file.
   * @return True on success, false on failure.
   */
  bool saveMap(const std::string & path);

  /**
   * @brief Load keyframes and data from a SQLite .map file.
   *
   * Poses are loaded immediately into memory. Data blobs may be loaded
   * lazily via the retained SQLite handle.
   * @param path Filesystem path of the .map file to load.
   * @return True on success, false on failure.
   */
  bool loadMap(const std::string & path);

  /// @brief Check whether a prior map has been loaded.
  /// @return True if loadMap() succeeded previously.
  bool hasPriorMap() const;

  /**
   * @brief Check whether a GTSAM key belongs to the loaded prior map.
   * @param key The GTSAM key to check.
   * @return True if the key came from a prior map load.
   */
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
