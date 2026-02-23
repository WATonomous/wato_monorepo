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
 */
class MapManager {
public:
  MapManager();

  // ---- Type registry ----

  struct TypeHandler {
    std::function<void(const std::any& data, const std::string& path)> serialize;
    std::function<std::any(const std::string& path)> deserialize;
  };

  /// Register a keyframe data type with serialize/deserialize functions.
  /// Called by plugins during onInitialize().
  void registerType(const std::string& key, TypeHandler handler);

  /// Check if a type is registered.
  bool hasType(const std::string& key) const;

  /// Get all registered keys for a given plugin name prefix.
  /// e.g., getKeysForPlugin("lidar_kep_factor") returns
  ///   ["lidar_kep_factor/corners", "lidar_kep_factor/surfaces"]
  std::vector<std::string> getKeysForPlugin(const std::string& plugin_name) const;

  // ---- Keyframe pose management ----

  /// Add a keyframe (pose only). Plugins store their own data via addKeyframeData().
  void addKeyframe(int index, const PoseType& pose);

  /// Update all keyframe poses after optimization.
  void updatePoses(const gtsam::Values& optimized);

  pcl::PointCloud<PointType>::Ptr getKeyPoses3D() const;
  pcl::PointCloud<PoseType>::Ptr getKeyPoses6D() const;
  int numKeyframes() const;

  // ---- Generic per-keyframe data storage ----

  /// Store data for a keyframe under a registered key.
  void addKeyframeData(int index, const std::string& key, std::any data);

  /// Retrieve data for a keyframe under a key.
  /// Returns std::nullopt if the key doesn't exist for this keyframe.
  std::optional<std::any> getKeyframeData(int index, const std::string& key) const;

  /// Retrieve all data for a keyframe from a specific plugin.
  /// Returns a map of key -> data for all keys matching the plugin prefix.
  std::unordered_map<std::string, std::any> getKeyframeDataForPlugin(
      int index, const std::string& plugin_name) const;

  // ---- Persistence ----

  /// Save the map to disk.
  bool saveMap(const std::string& directory, float resolution,
               const gtsam::NonlinearFactorGraph& graph,
               const gtsam::Values& values);

  /// Load a map from disk.
  bool loadMap(const std::string& directory);

  /// Check if a prior map has been loaded.
  bool hasPriorMap() const;

private:
  // Poses
  pcl::PointCloud<PointType>::Ptr key_poses_3d_;
  pcl::PointCloud<PoseType>::Ptr key_poses_6d_;

  // Type registry: key -> handler
  std::unordered_map<std::string, TypeHandler> type_handlers_;

  // Per-keyframe data: keyframe_index -> key -> data
  std::vector<std::unordered_map<std::string, std::any>> keyframe_data_;

  bool prior_map_loaded_ = false;
  mutable std::mutex mtx_;
};

}  // namespace eidos
