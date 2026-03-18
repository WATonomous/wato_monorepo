#include "eidos/core/map_manager.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <gtsam/inference/Symbol.h>

namespace eidos {

MapManager::MapManager() {
  key_poses_3d_ = pcl::make_shared<pcl::PointCloud<PointType>>();
  key_poses_6d_ = pcl::make_shared<pcl::PointCloud<PoseType>>();
  kdtree_ = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
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
  kdtree_dirty_ = true;
}

std::string MapManager::getOwnerPlugin(gtsam::Key gtsam_key) const {
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = key_owner_plugin_.find(gtsam_key);
  return (it != key_owner_plugin_.end()) ? it->second : "";
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
  return key_poses_3d_;
}

pcl::PointCloud<PoseType>::Ptr MapManager::getKeyPoses6D() const {
  return key_poses_6d_;
}

pcl::KdTreeFLANN<PointType>::Ptr MapManager::getKdTree() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (kdtree_dirty_ && !key_poses_3d_->empty()) {
    kdtree_->setInputCloud(key_poses_3d_);
    kdtree_dirty_ = false;
  }
  return kdtree_;
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
  return (it != key_to_cloud_index_.end()) ? it->second : -1;
}

gtsam::Key MapManager::getKeyFromCloudIndex(int cloud_index) const {
  std::lock_guard<std::mutex> lock(mtx_);
  for (const auto& [key, idx] : key_to_cloud_index_) {
    if (idx == cloud_index) return key;
  }
  return 0;
}

// ---- Per-keyframe data storage ----

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

// ---- Graph adjacency ----

void MapManager::addEdges(const gtsam::NonlinearFactorGraph& new_factors,
                           const std::vector<std::string>& factor_owners) {
  std::lock_guard<std::mutex> lock(mtx_);
  for (size_t i = 0; i < new_factors.size(); i++) {
    auto factor = new_factors[i];
    if (!factor) continue;
    auto keys = factor->keys();
    std::string owner = (i < factor_owners.size()) ? factor_owners[i] : "";
    for (size_t a = 0; a < keys.size(); a++) {
      for (size_t b = a + 1; b < keys.size(); b++) {
        adjacency_[keys[a]].push_back(keys[b]);
        adjacency_[keys[b]].push_back(keys[a]);
        auto edge_key = std::make_pair(std::min(keys[a], keys[b]),
                                        std::max(keys[a], keys[b]));
        if (!owner.empty()) edge_owners_[edge_key] = owner;
      }
    }
  }
}

std::string MapManager::getEdgeOwner(gtsam::Key key_a, gtsam::Key key_b) const {
  std::lock_guard<std::mutex> lock(mtx_);
  auto edge_key = std::make_pair(std::min(key_a, key_b), std::max(key_a, key_b));
  auto it = edge_owners_.find(edge_key);
  return (it != edge_owners_.end()) ? it->second : "";
}

const std::unordered_map<gtsam::Key, std::vector<gtsam::Key>>&
MapManager::getAdjacency() const {
  return adjacency_;
}

// ---- Persistence ----

bool MapManager::saveMap(const std::string& directory,
                         const std::vector<std::string>& plugin_names) {
  std::lock_guard<std::mutex> lock(mtx_);
  namespace fs = std::filesystem;
  fs::create_directories(directory);

  // Save 6DOF poses
  pcl::io::savePCDFileBinary(directory + "/poses.pcd", *key_poses_6d_);

  // Save keys + owner plugin names
  {
    std::ofstream ofs(directory + "/keys.bin", std::ios::binary);
    uint32_t num = static_cast<uint32_t>(key_list_.size());
    ofs.write(reinterpret_cast<const char*>(&num), sizeof(num));
    for (size_t i = 0; i < key_list_.size(); i++) {
      uint64_t k = key_list_[i];
      ofs.write(reinterpret_cast<const char*>(&k), sizeof(k));
      std::string owner;
      auto it = key_owner_plugin_.find(key_list_[i]);
      if (it != key_owner_plugin_.end()) owner = it->second;
      uint16_t len = static_cast<uint16_t>(owner.size());
      ofs.write(reinterpret_cast<const char*>(&len), sizeof(len));
      if (len > 0) ofs.write(owner.data(), len);
    }
  }

  // Save metadata
  {
    std::ofstream meta(directory + "/metadata.yaml");
    meta << "version: 3\n";
    meta << "num_states: " << static_cast<int>(key_list_.size()) << "\n";
    meta << "plugins:\n";
    for (const auto& name : plugin_names) {
      meta << "  - \"" << name << "\"\n";
    }
  }

  return true;
}

bool MapManager::loadMap(const std::string& directory,
                         std::vector<std::string>& saved_plugin_names) {
  std::lock_guard<std::mutex> lock(mtx_);
  namespace fs = std::filesystem;

  if (!fs::exists(directory)) return false;

  auto poses_path = fs::path(directory) / "poses.pcd";
  auto keys_path = fs::path(directory) / "keys.bin";
  if (!fs::exists(poses_path)) return false;

  // Clear existing state
  key_poses_3d_->clear();
  key_poses_6d_->clear();
  keyframe_data_.clear();
  key_to_cloud_index_.clear();
  key_list_.clear();
  key_owner_plugin_.clear();
  global_data_.clear();
  adjacency_.clear();

  // Load poses
  pcl::io::loadPCDFile(poses_path.string(), *key_poses_6d_);
  int num_states = static_cast<int>(key_poses_6d_->size());

  // Rebuild 3D poses from 6D
  key_poses_3d_->resize(num_states);
  for (int i = 0; i < num_states; i++) {
    key_poses_3d_->points[i].x = key_poses_6d_->points[i].x;
    key_poses_3d_->points[i].y = key_poses_6d_->points[i].y;
    key_poses_3d_->points[i].z = key_poses_6d_->points[i].z;
    key_poses_3d_->points[i].intensity = static_cast<float>(i);
    key_poses_6d_->points[i].intensity = static_cast<float>(i);
  }

  // Load keys
  if (fs::exists(keys_path)) {
    std::ifstream ifs(keys_path.string(), std::ios::binary);
    uint32_t num = 0;
    ifs.read(reinterpret_cast<char*>(&num), sizeof(num));
    for (uint32_t i = 0; i < num && i < static_cast<uint32_t>(num_states); i++) {
      uint64_t k = 0;
      ifs.read(reinterpret_cast<char*>(&k), sizeof(k));
      uint16_t len = 0;
      ifs.read(reinterpret_cast<char*>(&len), sizeof(len));
      std::string owner(len, '\0');
      if (len > 0) ifs.read(owner.data(), len);

      gtsam::Key gtsam_key = k;
      key_to_cloud_index_[gtsam_key] = static_cast<int>(i);
      key_list_.push_back(gtsam_key);
      if (!owner.empty()) key_owner_plugin_[gtsam_key] = owner;
    }
  } else {
    // Legacy: no keys.bin, create sequential keys
    for (int i = 0; i < num_states; i++) {
      gtsam::Key gtsam_key = gtsam::Symbol(255, i);
      key_to_cloud_index_[gtsam_key] = i;
      key_list_.push_back(gtsam_key);
    }
  }

  // Read metadata for plugin names
  auto meta_path = fs::path(directory) / "metadata.yaml";
  if (fs::exists(meta_path)) {
    std::ifstream meta(meta_path.string());
    std::string line;
    bool in_plugins = false;
    while (std::getline(meta, line)) {
      if (line.find("plugins:") != std::string::npos) {
        in_plugins = true;
        continue;
      }
      if (in_plugins) {
        auto dash = line.find("- ");
        if (dash == std::string::npos) { in_plugins = false; continue; }
        std::string val = line.substr(dash + 2);
        if (val.size() >= 2 && val.front() == '"' && val.back() == '"')
          val = val.substr(1, val.size() - 2);
        if (!val.empty()) saved_plugin_names.push_back(val);
      }
    }
  }

  // Track which keys are from the prior map (not in ISAM2)
  prior_map_keys_.clear();
  for (auto k : key_list_) {
    prior_map_keys_.insert(k);
  }

  kdtree_dirty_ = true;
  prior_map_loaded_ = true;
  return true;
}

bool MapManager::isPriorMapKey(gtsam::Key key) const {
  std::lock_guard<std::mutex> lock(mtx_);
  return prior_map_keys_.count(key) > 0;
}

bool MapManager::hasPriorMap() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return prior_map_loaded_;
}

}  // namespace eidos
