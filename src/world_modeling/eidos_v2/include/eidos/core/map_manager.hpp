#pragma once

#include <any>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "eidos/utils/types.hpp"

namespace eidos {

/**
 * @brief Centralized keyframe pose + data store.
 *
 * Owns keyframe poses (PCL clouds for KD-tree) and provides generic
 * per-keyframe and global data storage via std::any.
 *
 * MapManager owns all persistence — saves/loads poses, keys, metadata,
 * and plugin data via registered serializers. Plugins register their
 * serializers in onInitialize(); MapManager handles all file I/O.
 *
 * TODO: Implement registerSerializer<T>() to replace per-plugin saveData/loadData.
 */
class MapManager {
public:
  MapManager();

  // ---- Keyframe pose management ----

  void addKeyframe(gtsam::Key gtsam_key, const PoseType& pose,
                   const std::string& owner = "");
  std::string getOwnerPlugin(gtsam::Key gtsam_key) const;
  void updatePoses(const gtsam::Values& optimized);

  pcl::PointCloud<PointType>::Ptr getKeyPoses3D() const;
  pcl::PointCloud<PoseType>::Ptr getKeyPoses6D() const;

  /// Cached KD-tree on key_poses_3d_, rebuilt lazily on first access after changes.
  pcl::KdTreeFLANN<PointType>::Ptr getKdTree();
  int numKeyframes() const;
  std::vector<gtsam::Key> getKeyList() const;
  int getCloudIndex(gtsam::Key gtsam_key) const;
  gtsam::Key getKeyFromCloudIndex(int cloud_index) const;

  // ---- Generic per-keyframe data storage (runtime) ----

  void addKeyframeData(gtsam::Key gtsam_key, const std::string& key, std::any data);
  std::optional<std::any> getKeyframeData(gtsam::Key gtsam_key, const std::string& key) const;
  std::unordered_map<std::string, std::any> getKeyframeDataForPlugin(
      gtsam::Key gtsam_key, const std::string& plugin_name) const;

  // ---- Global data storage (runtime) ----

  void setGlobalData(const std::string& key, std::any data);
  std::optional<std::any> getGlobalData(const std::string& key) const;

  // ---- Graph adjacency (incremental) ----

  void addEdges(const gtsam::NonlinearFactorGraph& new_factors,
                const std::vector<std::string>& factor_owners = {});
  const std::unordered_map<gtsam::Key, std::vector<gtsam::Key>>& getAdjacency() const;

  /// Get the owner plugin name for an edge (key_a, key_b). Returns "" if unknown.
  std::string getEdgeOwner(gtsam::Key key_a, gtsam::Key key_b) const;

  // ---- Persistence (poses + keys + metadata only) ----

  /// Save poses and keys to directory. Returns list of plugin names found.
  bool saveMap(const std::string& directory,
               const std::vector<std::string>& plugin_names);
  /// Load poses and keys from directory. Returns plugin names from metadata.
  bool loadMap(const std::string& directory,
               std::vector<std::string>& saved_plugin_names);
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
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::string> edge_owners_; ///< (min_key, max_key) → owner

  pcl::KdTreeFLANN<PointType>::Ptr kdtree_;
  bool kdtree_dirty_ = true;

  std::set<gtsam::Key> prior_map_keys_;

  bool prior_map_loaded_ = false;
  mutable std::mutex mtx_;
};

}  // namespace eidos
