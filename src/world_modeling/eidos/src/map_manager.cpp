#include "eidos/map_manager.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/serialization.h>

namespace eidos {

MapManager::MapManager() {
  key_poses_3d_ = pcl::make_shared<pcl::PointCloud<PointType>>();
  key_poses_6d_ = pcl::make_shared<pcl::PointCloud<PoseType>>();
}

// ---- Type registry ----

void MapManager::registerType(const std::string& key, TypeHandler handler) {
  std::lock_guard<std::mutex> lock(mtx_);
  type_handlers_[key] = std::move(handler);
}

bool MapManager::hasType(const std::string& key) const {
  std::lock_guard<std::mutex> lock(mtx_);
  return type_handlers_.count(key) > 0;
}

std::vector<std::string> MapManager::getKeysForPlugin(
    const std::string& plugin_name) const {
  std::lock_guard<std::mutex> lock(mtx_);
  std::string prefix = plugin_name + "/";
  std::vector<std::string> keys;
  for (const auto& [key, _] : type_handlers_) {
    if (key.rfind(prefix, 0) == 0) {
      keys.push_back(key);
    }
  }
  std::sort(keys.begin(), keys.end());
  return keys;
}

// ---- Keyframe pose management ----

void MapManager::addKeyframe(int index, const PoseType& pose) {
  std::lock_guard<std::mutex> lock(mtx_);

  PointType pose_3d;
  pose_3d.x = pose.x;
  pose_3d.y = pose.y;
  pose_3d.z = pose.z;
  pose_3d.intensity = static_cast<float>(index);
  key_poses_3d_->push_back(pose_3d);

  PoseType pose_6d = pose;
  pose_6d.intensity = static_cast<float>(index);
  key_poses_6d_->push_back(pose_6d);

  // Ensure keyframe_data_ has enough slots
  if (static_cast<int>(keyframe_data_.size()) <= index) {
    keyframe_data_.resize(index + 1);
  }
}

void MapManager::updatePoses(const gtsam::Values& optimized) {
  std::lock_guard<std::mutex> lock(mtx_);

  int num_poses = static_cast<int>(key_poses_3d_->size());
  for (int i = 0; i < num_poses; i++) {
    auto pose = optimized.at<gtsam::Pose3>(i);
    key_poses_3d_->points[i].x = static_cast<float>(pose.translation().x());
    key_poses_3d_->points[i].y = static_cast<float>(pose.translation().y());
    key_poses_3d_->points[i].z = static_cast<float>(pose.translation().z());

    key_poses_6d_->points[i].x = key_poses_3d_->points[i].x;
    key_poses_6d_->points[i].y = key_poses_3d_->points[i].y;
    key_poses_6d_->points[i].z = key_poses_3d_->points[i].z;
    key_poses_6d_->points[i].roll = static_cast<float>(pose.rotation().roll());
    key_poses_6d_->points[i].pitch = static_cast<float>(pose.rotation().pitch());
    key_poses_6d_->points[i].yaw = static_cast<float>(pose.rotation().yaw());
  }
}

pcl::PointCloud<PointType>::Ptr MapManager::getKeyPoses3D() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return key_poses_3d_;
}

pcl::PointCloud<PoseType>::Ptr MapManager::getKeyPoses6D() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return key_poses_6d_;
}

int MapManager::numKeyframes() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return static_cast<int>(key_poses_3d_->size());
}

// ---- Generic per-keyframe data storage ----

void MapManager::addKeyframeData(
    int index, const std::string& key, std::any data) {
  std::lock_guard<std::mutex> lock(mtx_);

  if (static_cast<int>(keyframe_data_.size()) <= index) {
    keyframe_data_.resize(index + 1);
  }
  keyframe_data_[index][key] = std::move(data);
}

std::optional<std::any> MapManager::getKeyframeData(
    int index, const std::string& key) const {
  std::lock_guard<std::mutex> lock(mtx_);

  if (index < 0 || index >= static_cast<int>(keyframe_data_.size())) {
    return std::nullopt;
  }
  auto it = keyframe_data_[index].find(key);
  if (it == keyframe_data_[index].end()) {
    return std::nullopt;
  }
  return it->second;
}

std::unordered_map<std::string, std::any> MapManager::getKeyframeDataForPlugin(
    int index, const std::string& plugin_name) const {
  std::lock_guard<std::mutex> lock(mtx_);

  std::unordered_map<std::string, std::any> result;
  if (index < 0 || index >= static_cast<int>(keyframe_data_.size())) {
    return result;
  }

  std::string prefix = plugin_name + "/";
  for (const auto& [key, data] : keyframe_data_[index]) {
    if (key.rfind(prefix, 0) == 0) {
      result[key] = data;
    }
  }
  return result;
}

// ---- Persistence ----

bool MapManager::saveMap(
    const std::string& directory, float resolution,
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values) {
  std::lock_guard<std::mutex> lock(mtx_);

  namespace fs = std::filesystem;
  fs::create_directories(directory);
  fs::create_directories(fs::path(directory) / "keyframes");

  // Save trajectory and transformations
  pcl::io::savePCDFileBinary(directory + "/trajectory.pcd", *key_poses_3d_);
  pcl::io::savePCDFileBinary(directory + "/transformations.pcd", *key_poses_6d_);

  // Save GTSAM factor graph and values
  try {
    gtsam::serializeToBinaryFile(graph, directory + "/graph_factors.bin");
    gtsam::serializeToBinaryFile(values, directory + "/graph_values.bin");
  } catch (const std::exception&) {
    // Serialization may not be available in all GTSAM builds;
    // fall back to just saving PCD files.
  }

  // Collect all registered keys that have data
  std::vector<std::string> registered_keys;
  for (const auto& [key, _] : type_handlers_) {
    registered_keys.push_back(key);
  }
  std::sort(registered_keys.begin(), registered_keys.end());

  // Save per-keyframe data using registered serializers
  int num_keyframes = static_cast<int>(keyframe_data_.size());
  for (int i = 0; i < num_keyframes; i++) {
    char dirname[32];
    snprintf(dirname, sizeof(dirname), "%06d", i);
    auto keyframe_dir = fs::path(directory) / "keyframes" / dirname;

    for (const auto& [key, data] : keyframe_data_[i]) {
      auto handler_it = type_handlers_.find(key);
      if (handler_it == type_handlers_.end()) continue;

      // Parse "plugin_name/data_name" into subdirectory and filename
      auto slash_pos = key.find('/');
      std::string plugin_subdir = key.substr(0, slash_pos);
      std::string data_name = key.substr(slash_pos + 1) + ".bin";

      auto data_dir = keyframe_dir / plugin_subdir;
      fs::create_directories(data_dir);
      std::string data_path = (data_dir / data_name).string();

      try {
        handler_it->second.serialize(data, data_path);
      } catch (const std::exception&) {
        continue;
      }
    }
  }

  // Save global map (combined point clouds from all registered pcl types)
  // This is a best-effort convenience output for visualization
  if (resolution > 0) {
    pcl::PointCloud<PointType>::Ptr global_map(
        new pcl::PointCloud<PointType>());

    for (int i = 0; i < num_keyframes; i++) {
      for (const auto& [key, data] : keyframe_data_[i]) {
        try {
          auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
          if (cloud && !cloud->empty()) {
            *global_map += *transformPointCloud(cloud, key_poses_6d_->points[i]);
          }
        } catch (const std::bad_any_cast&) {
          // Not a point cloud type, skip
        }
      }
    }

    if (!global_map->empty()) {
      pcl::VoxelGrid<PointType> filter;
      pcl::PointCloud<PointType>::Ptr ds(new pcl::PointCloud<PointType>());
      filter.setLeafSize(resolution, resolution, resolution);
      filter.setInputCloud(global_map);
      filter.filter(*ds);
      pcl::io::savePCDFileBinary(directory + "/GlobalMap.pcd", *ds);
    }
  }

  // Save metadata
  {
    std::ofstream meta(directory + "/metadata.yaml");
    meta << "version: 1\n";
    meta << "num_keyframes: " << num_keyframes << "\n";
    meta << "resolution: " << resolution << "\n";
    meta << "registered_keys:\n";
    for (const auto& key : registered_keys) {
      meta << "  - \"" << key << "\"\n";
    }
    meta.close();
  }

  return true;
}

bool MapManager::loadMap(const std::string& directory) {
  std::lock_guard<std::mutex> lock(mtx_);

  namespace fs = std::filesystem;
  if (!fs::exists(directory)) return false;

  // Load trajectory and transformations
  auto traj_path = fs::path(directory) / "trajectory.pcd";
  auto trans_path = fs::path(directory) / "transformations.pcd";

  if (!fs::exists(traj_path) || !fs::exists(trans_path)) return false;

  key_poses_3d_->clear();
  key_poses_6d_->clear();
  keyframe_data_.clear();

  pcl::io::loadPCDFile(traj_path.string(), *key_poses_3d_);
  pcl::io::loadPCDFile(trans_path.string(), *key_poses_6d_);

  int num_keyframes = static_cast<int>(key_poses_3d_->size());
  keyframe_data_.resize(num_keyframes);

  // Read metadata to get the list of registered keys
  auto metadata_path = fs::path(directory) / "metadata.yaml";
  std::vector<std::string> saved_keys;
  if (fs::exists(metadata_path)) {
    std::ifstream meta(metadata_path.string());
    std::string line;
    bool in_keys_section = false;
    while (std::getline(meta, line)) {
      if (line.find("registered_keys:") != std::string::npos) {
        in_keys_section = true;
        continue;
      }
      if (in_keys_section) {
        // Lines look like:   - "plugin_name/data_name"
        auto dash_pos = line.find("- ");
        if (dash_pos == std::string::npos) {
          in_keys_section = false;
          continue;
        }
        std::string value = line.substr(dash_pos + 2);
        // Strip quotes
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
          value = value.substr(1, value.size() - 2);
        }
        if (!value.empty()) {
          saved_keys.push_back(value);
        }
      }
    }
  }

  // Load per-keyframe data using registered deserializers
  auto keyframes_dir = fs::path(directory) / "keyframes";
  if (fs::exists(keyframes_dir)) {
    for (int i = 0; i < num_keyframes; i++) {
      char dirname[32];
      snprintf(dirname, sizeof(dirname), "%06d", i);
      auto keyframe_dir = keyframes_dir / dirname;
      if (!fs::exists(keyframe_dir)) continue;

      for (const auto& key : saved_keys) {
        auto handler_it = type_handlers_.find(key);
        if (handler_it == type_handlers_.end()) {
          // No deserializer registered for this key (plugin not loaded)
          continue;
        }

        auto slash_pos = key.find('/');
        std::string plugin_subdir = key.substr(0, slash_pos);
        std::string data_name = key.substr(slash_pos + 1) + ".bin";
        auto data_path = keyframe_dir / plugin_subdir / data_name;

        if (!fs::exists(data_path)) continue;

        try {
          auto data = handler_it->second.deserialize(data_path.string());
          keyframe_data_[i][key] = std::move(data);
        } catch (const std::exception&) {
          continue;
        }
      }
    }
  }

  prior_map_loaded_ = true;
  return true;
}

bool MapManager::hasPriorMap() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return prior_map_loaded_;
}

}  // namespace eidos
