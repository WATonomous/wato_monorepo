#include "eidos/map_manager.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <gtsam/inference/Symbol.h>
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

void MapManager::addKeyframe(gtsam::Key gtsam_key, const PoseType& pose,
                             const std::string& owner) {
  std::lock_guard<std::mutex> lock(mtx_);

  int cloud_index = static_cast<int>(key_poses_3d_->size());

  PointType pose_3d;
  pose_3d.x = pose.x;
  pose_3d.y = pose.y;
  pose_3d.z = pose.z;
  pose_3d.intensity = static_cast<float>(cloud_index);
  key_poses_3d_->push_back(pose_3d);

  PoseType pose_6d = pose;
  pose_6d.intensity = static_cast<float>(cloud_index);
  key_poses_6d_->push_back(pose_6d);

  key_to_cloud_index_[gtsam_key] = cloud_index;
  key_list_.push_back(gtsam_key);
  if (!owner.empty()) {
    key_owner_plugin_[gtsam_key] = owner;
  }
}

std::string MapManager::getOwnerPlugin(gtsam::Key gtsam_key) const {
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = key_owner_plugin_.find(gtsam_key);
  if (it == key_owner_plugin_.end()) return "";
  return it->second;
}

void MapManager::updatePoses(const gtsam::Values& optimized) {
  std::lock_guard<std::mutex> lock(mtx_);

  for (const auto& [gtsam_key, cloud_idx] : key_to_cloud_index_) {
    if (!optimized.exists(gtsam_key)) continue;

    auto pose = optimized.at<gtsam::Pose3>(gtsam_key);
    key_poses_3d_->points[cloud_idx].x = static_cast<float>(pose.translation().x());
    key_poses_3d_->points[cloud_idx].y = static_cast<float>(pose.translation().y());
    key_poses_3d_->points[cloud_idx].z = static_cast<float>(pose.translation().z());

    key_poses_6d_->points[cloud_idx].x = key_poses_3d_->points[cloud_idx].x;
    key_poses_6d_->points[cloud_idx].y = key_poses_3d_->points[cloud_idx].y;
    key_poses_6d_->points[cloud_idx].z = key_poses_3d_->points[cloud_idx].z;
    key_poses_6d_->points[cloud_idx].roll = static_cast<float>(pose.rotation().roll());
    key_poses_6d_->points[cloud_idx].pitch = static_cast<float>(pose.rotation().pitch());
    key_poses_6d_->points[cloud_idx].yaw = static_cast<float>(pose.rotation().yaw());
  }
}

pcl::PointCloud<PointType>::Ptr MapManager::getKeyPoses3D() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return pcl::make_shared<pcl::PointCloud<PointType>>(*key_poses_3d_);
}

pcl::PointCloud<PoseType>::Ptr MapManager::getKeyPoses6D() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return pcl::make_shared<pcl::PointCloud<PoseType>>(*key_poses_6d_);
}

int MapManager::numKeyframes() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return static_cast<int>(key_poses_3d_->size());
}

std::vector<gtsam::Key> MapManager::getKeyList() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return key_list_;
}

int MapManager::getCloudIndex(gtsam::Key gtsam_key) const {
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = key_to_cloud_index_.find(gtsam_key);
  if (it == key_to_cloud_index_.end()) return -1;
  return it->second;
}

gtsam::Key MapManager::getKeyFromCloudIndex(int cloud_index) const {
  std::lock_guard<std::mutex> lock(mtx_);
  if (cloud_index < 0 || cloud_index >= static_cast<int>(key_list_.size()))
    return 0;
  return key_list_[cloud_index];
}

// ---- Generic per-keyframe data storage ----

void MapManager::addKeyframeData(
    gtsam::Key gtsam_key, const std::string& key, std::any data) {
  std::lock_guard<std::mutex> lock(mtx_);
  keyframe_data_[gtsam_key][key] = std::move(data);
}

std::optional<std::any> MapManager::getKeyframeData(
    gtsam::Key gtsam_key, const std::string& key) const {
  std::lock_guard<std::mutex> lock(mtx_);

  auto kf_it = keyframe_data_.find(gtsam_key);
  if (kf_it == keyframe_data_.end()) return std::nullopt;

  auto it = kf_it->second.find(key);
  if (it == kf_it->second.end()) return std::nullopt;

  return it->second;
}

std::unordered_map<std::string, std::any> MapManager::getKeyframeDataForPlugin(
    gtsam::Key gtsam_key, const std::string& plugin_name) const {
  std::lock_guard<std::mutex> lock(mtx_);

  std::unordered_map<std::string, std::any> result;
  auto kf_it = keyframe_data_.find(gtsam_key);
  if (kf_it == keyframe_data_.end()) return result;

  std::string prefix = plugin_name + "/";
  for (const auto& [key, data] : kf_it->second) {
    if (key.rfind(prefix, 0) == 0) {
      result[key] = data;
    }
  }
  return result;
}

// ---- Global data storage ----

void MapManager::registerGlobalType(const std::string& key, TypeHandler handler) {
  std::lock_guard<std::mutex> lock(mtx_);
  global_type_handlers_[key] = std::move(handler);
}

void MapManager::setGlobalData(const std::string& key, std::any data) {
  std::lock_guard<std::mutex> lock(mtx_);
  global_data_[key] = std::move(data);
}

std::optional<std::any> MapManager::getGlobalData(const std::string& key) const {
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = global_data_.find(key);
  if (it == global_data_.end()) return std::nullopt;
  return it->second;
}

// ---- Graph adjacency (incremental) ----

void MapManager::addEdges(const gtsam::NonlinearFactorGraph& new_factors) {
  std::lock_guard<std::mutex> lock(mtx_);
  for (size_t i = 0; i < new_factors.size(); i++) {
    auto factor = new_factors[i];
    if (!factor) continue;
    auto keys = factor->keys();
    for (size_t a = 0; a < keys.size(); a++) {
      for (size_t b = a + 1; b < keys.size(); b++) {
        adjacency_[keys[a]].push_back(keys[b]);
        adjacency_[keys[b]].push_back(keys[a]);
      }
    }
  }
}

const std::unordered_map<gtsam::Key, std::vector<gtsam::Key>>&
MapManager::getAdjacency() const {
  // Caller is expected to hold no lock — this is safe because addEdges()
  // only appends (vectors grow, never shrink) and BFS reads are on the
  // SLAM thread which serialises with addEdges via slamLoop's mtx_.
  return adjacency_;
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
    // Serialization may not be available in all GTSAM builds
  }

  // Collect all registered keys
  std::vector<std::string> registered_keys;
  for (const auto& [key, _] : type_handlers_) {
    registered_keys.push_back(key);
  }
  std::sort(registered_keys.begin(), registered_keys.end());

  // Save per-keyframe data using registered serializers
  int kf_idx = 0;
  for (const auto& gtsam_key : key_list_) {
    char dirname[32];
    snprintf(dirname, sizeof(dirname), "%06d", kf_idx);
    auto keyframe_dir = fs::path(directory) / "keyframes" / dirname;

    auto kf_it = keyframe_data_.find(gtsam_key);
    if (kf_it != keyframe_data_.end()) {
      for (const auto& [key, data] : kf_it->second) {
        auto handler_it = type_handlers_.find(key);
        if (handler_it == type_handlers_.end()) continue;

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
    kf_idx++;
  }

  // Save global data
  auto global_dir = fs::path(directory) / "global";
  fs::create_directories(global_dir);
  std::vector<std::string> global_keys;
  for (const auto& [key, data] : global_data_) {
    auto handler_it = global_type_handlers_.find(key);
    if (handler_it == global_type_handlers_.end()) continue;

    std::string filename = key;
    std::replace(filename.begin(), filename.end(), '/', '_');
    std::string data_path = (global_dir / (filename + ".bin")).string();

    try {
      handler_it->second.serialize(data, data_path);
      global_keys.push_back(key);
    } catch (const std::exception&) {
      continue;
    }
  }

  // Save global map (combined point clouds)
  if (resolution > 0) {
    pcl::PointCloud<PointType>::Ptr global_map(
        new pcl::PointCloud<PointType>());

    int idx = 0;
    for (const auto& gtsam_key : key_list_) {
      auto kf_it = keyframe_data_.find(gtsam_key);
      if (kf_it == keyframe_data_.end()) { idx++; continue; }

      for (const auto& [key, data] : kf_it->second) {
        try {
          auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
          if (cloud && !cloud->empty()) {
            *global_map += *transformPointCloud(cloud, key_poses_6d_->points[idx]);
          }
        } catch (const std::bad_any_cast&) {
        }
      }
      idx++;
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
    meta << "version: 2\n";
    meta << "num_keyframes: " << static_cast<int>(key_list_.size()) << "\n";
    meta << "resolution: " << resolution << "\n";
    meta << "registered_keys:\n";
    for (const auto& key : registered_keys) {
      meta << "  - \"" << key << "\"\n";
    }
    meta << "global_keys:\n";
    for (const auto& key : global_keys) {
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

  auto traj_path = fs::path(directory) / "trajectory.pcd";
  auto trans_path = fs::path(directory) / "transformations.pcd";

  if (!fs::exists(traj_path) || !fs::exists(trans_path)) return false;

  key_poses_3d_->clear();
  key_poses_6d_->clear();
  keyframe_data_.clear();
  key_to_cloud_index_.clear();
  key_list_.clear();

  pcl::io::loadPCDFile(traj_path.string(), *key_poses_3d_);
  pcl::io::loadPCDFile(trans_path.string(), *key_poses_6d_);

  int num_keyframes = static_cast<int>(key_poses_3d_->size());

  // For legacy maps (version 1), create sequential keys using index 255
  // (the prior/init symbol index). Future maps will store the actual keys.
  for (int i = 0; i < num_keyframes; i++) {
    gtsam::Key gtsam_key = gtsam::Symbol(255, i);
    key_to_cloud_index_[gtsam_key] = i;
    key_list_.push_back(gtsam_key);
  }

  // Read metadata
  auto metadata_path = fs::path(directory) / "metadata.yaml";
  std::vector<std::string> saved_keys;
  std::vector<std::string> saved_global_keys;
  if (fs::exists(metadata_path)) {
    std::ifstream meta(metadata_path.string());
    std::string line;
    enum Section { NONE, REGISTERED_KEYS, GLOBAL_KEYS } section = NONE;
    while (std::getline(meta, line)) {
      if (line.find("registered_keys:") != std::string::npos) {
        section = REGISTERED_KEYS;
        continue;
      }
      if (line.find("global_keys:") != std::string::npos) {
        section = GLOBAL_KEYS;
        continue;
      }
      if (section == REGISTERED_KEYS || section == GLOBAL_KEYS) {
        auto dash_pos = line.find("- ");
        if (dash_pos == std::string::npos) {
          section = NONE;
          continue;
        }
        std::string value = line.substr(dash_pos + 2);
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
          value = value.substr(1, value.size() - 2);
        }
        if (!value.empty()) {
          if (section == REGISTERED_KEYS) {
            saved_keys.push_back(value);
          } else {
            saved_global_keys.push_back(value);
          }
        }
      }
    }
  }

  // Load per-keyframe data
  auto keyframes_dir = fs::path(directory) / "keyframes";
  if (fs::exists(keyframes_dir)) {
    for (int i = 0; i < num_keyframes; i++) {
      char dirname[32];
      snprintf(dirname, sizeof(dirname), "%06d", i);
      auto keyframe_dir = keyframes_dir / dirname;
      if (!fs::exists(keyframe_dir)) continue;

      gtsam::Key gtsam_key = key_list_[i];

      for (const auto& key : saved_keys) {
        auto handler_it = type_handlers_.find(key);
        if (handler_it == type_handlers_.end()) continue;

        auto slash_pos = key.find('/');
        std::string plugin_subdir = key.substr(0, slash_pos);
        std::string data_name = key.substr(slash_pos + 1) + ".bin";
        auto data_path = keyframe_dir / plugin_subdir / data_name;

        if (!fs::exists(data_path)) continue;

        try {
          auto data = handler_it->second.deserialize(data_path.string());
          keyframe_data_[gtsam_key][key] = std::move(data);
        } catch (const std::exception&) {
          continue;
        }
      }
    }
  }

  // Load global data
  auto global_dir = fs::path(directory) / "global";
  if (fs::exists(global_dir)) {
    for (const auto& key : saved_global_keys) {
      auto handler_it = global_type_handlers_.find(key);
      if (handler_it == global_type_handlers_.end()) continue;

      std::string filename = key;
      std::replace(filename.begin(), filename.end(), '/', '_');
      auto data_path = global_dir / (filename + ".bin");

      if (!fs::exists(data_path)) continue;

      try {
        auto data = handler_it->second.deserialize(data_path.string());
        global_data_[key] = std::move(data);
      } catch (const std::exception&) {
        continue;
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
