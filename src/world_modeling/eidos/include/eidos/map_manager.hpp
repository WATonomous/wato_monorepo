#pragma once

#include <any>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief Generic keyframe data store with type registry.
 *
 * MapManager owns keyframe poses and provides a registry for plugins to
 * store/retrieve arbitrary per-keyframe data. It does not know about
 * point clouds, images, or any sensor-specific types.
 *
 * Plugins register their data types at initialization time. Each type is
 * identified by a string key (convention: "plugin_name/data_name") and
 * paired with serialize/deserialize functions.
 *
 * Keyframes are indexed by gtsam::Key (from Symbol(plugin_index, keygroup)).
 * The poses are stored both in PCL clouds (for efficient KD-tree queries)
 * and in a key→cloud_index map for random access.
 */
class MapManager {
public:
  MapManager();

  // ---- Type registry ----

  struct TypeHandler {
    std::function<void(const std::any& data, const std::string& path)> serialize;
    std::function<std::any(const std::string& path)> deserialize;
  };

  void registerType(const std::string& key, TypeHandler handler);
  bool hasType(const std::string& key) const;
  std::vector<std::string> getKeysForPlugin(const std::string& plugin_name) const;

  // ---- Keyframe pose management ----

  /// Add a keyframe. The gtsam_key is used for random access; poses are
  /// appended to the PCL clouds for KD-tree queries.
  void addKeyframe(gtsam::Key gtsam_key, const PoseType& pose);

  /// Update all keyframe poses after optimization. Uses stored key→index map.
  void updatePoses(const gtsam::Values& optimized);

  pcl::PointCloud<PointType>::Ptr getKeyPoses3D() const;
  pcl::PointCloud<PoseType>::Ptr getKeyPoses6D() const;
  int numKeyframes() const;

  /// Get ordered list of all GTSAM keys (insertion order).
  std::vector<gtsam::Key> getKeyList() const;

  /// Get the cloud index for a given GTSAM key (-1 if not found).
  int getCloudIndex(gtsam::Key gtsam_key) const;

  /// Get the GTSAM key for a given cloud index (0 if not found).
  gtsam::Key getKeyFromCloudIndex(int cloud_index) const;

  // ---- Generic per-keyframe data storage ----

  void addKeyframeData(gtsam::Key gtsam_key, const std::string& key, std::any data);
  std::optional<std::any> getKeyframeData(gtsam::Key gtsam_key, const std::string& key) const;
  std::unordered_map<std::string, std::any> getKeyframeDataForPlugin(
      gtsam::Key gtsam_key, const std::string& plugin_name) const;

  // ---- Global data storage ----

  void registerGlobalType(const std::string& key, TypeHandler handler);
  void setGlobalData(const std::string& key, std::any data);
  std::optional<std::any> getGlobalData(const std::string& key) const;

  // ---- Persistence ----

  bool saveMap(const std::string& directory, float resolution,
               const gtsam::NonlinearFactorGraph& graph,
               const gtsam::Values& values);
  bool loadMap(const std::string& directory);
  bool hasPriorMap() const;

private:
  // Poses (PCL clouds for KD-tree + sequential access)
  pcl::PointCloud<PointType>::Ptr key_poses_3d_;
  pcl::PointCloud<PoseType>::Ptr key_poses_6d_;

  // Key→cloud_index mapping for random access
  std::map<gtsam::Key, int> key_to_cloud_index_;
  std::vector<gtsam::Key> key_list_;  // insertion-ordered list of keys

  // Type registry: key -> handler
  std::unordered_map<std::string, TypeHandler> type_handlers_;

  // Per-keyframe data: gtsam_key -> key -> data
  std::map<gtsam::Key, std::unordered_map<std::string, std::any>> keyframe_data_;

  // Global data type registry and storage
  std::unordered_map<std::string, TypeHandler> global_type_handlers_;
  std::unordered_map<std::string, std::any> global_data_;

  bool prior_map_loaded_ = false;
  mutable std::mutex mtx_;
};

}  // namespace eidos
