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

#include "eidos/core/map_manager.hpp"

#include <gtsam/inference/Symbol.h>
#include <sqlite3.h>

#include <algorithm>
#include <cstring>
#include <filesystem>

#include "eidos/formats/registry.hpp"

namespace eidos
{

MapManager::MapManager()
{
  key_poses_3d_ = pcl::make_shared<pcl::PointCloud<PointType>>();
  key_poses_6d_ = pcl::make_shared<pcl::PointCloud<PoseType>>();
  kdtree_ = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
}

MapManager::~MapManager()
{
  if (load_db_) {
    sqlite3_close(load_db_);
    load_db_ = nullptr;
  }
}

// ==========================================================================
// Format registration
// ==========================================================================

void MapManager::registerKeyframeFormat(const std::string & data_key, const std::string & format)
{
  keyframe_formats_[data_key] = format;
}

void MapManager::registerGlobalFormat(const std::string & data_key, const std::string & format)
{
  global_formats_[data_key] = format;
}

// ==========================================================================
// Keyframe pose management (unchanged from v1)
// ==========================================================================

void MapManager::addKeyframe(gtsam::Key gtsam_key, const PoseType & pose, const std::string & owner)
{
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
  if (!owner.empty()) key_owner_plugin_[gtsam_key] = owner;
  kdtree_dirty_ = true;
}

std::string MapManager::getOwnerPlugin(gtsam::Key gtsam_key) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = key_owner_plugin_.find(gtsam_key);
  return (it != key_owner_plugin_.end()) ? it->second : "";
}

void MapManager::updatePoses(const gtsam::Values & optimized)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for (const auto & [gtsam_key, cloud_idx] : key_to_cloud_index_) {
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
  kdtree_dirty_ = true;
}

pcl::PointCloud<PointType>::Ptr MapManager::getKeyPoses3D() const
{
  return key_poses_3d_;
}

pcl::PointCloud<PoseType>::Ptr MapManager::getKeyPoses6D() const
{
  return key_poses_6d_;
}

pcl::KdTreeFLANN<PointType>::Ptr MapManager::getKdTree()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (kdtree_dirty_ && !key_poses_3d_->empty()) {
    kdtree_->setInputCloud(key_poses_3d_);
    kdtree_dirty_ = false;
  }
  return kdtree_;
}

int MapManager::numKeyframes() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return static_cast<int>(key_poses_3d_->size());
}

std::vector<gtsam::Key> MapManager::getKeyList() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return key_list_;
}

int MapManager::getCloudIndex(gtsam::Key gtsam_key) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = key_to_cloud_index_.find(gtsam_key);
  return (it != key_to_cloud_index_.end()) ? it->second : -1;
}

gtsam::Key MapManager::getKeyFromCloudIndex(int cloud_index) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  for (const auto & [key, idx] : key_to_cloud_index_) {
    if (idx == cloud_index) return key;
  }
  return 0;
}

// ==========================================================================
// Data storage helpers
// ==========================================================================

bool MapManager::hasKeyframeData(gtsam::Key key, const std::string & data_key) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto kf = keyframe_data_.find(key);
  if (kf == keyframe_data_.end()) return false;
  return kf->second.count(data_key) > 0;
}

// ==========================================================================
// Graph adjacency (unchanged)
// ==========================================================================

void MapManager::addEdges(
  const gtsam::NonlinearFactorGraph & new_factors, const std::vector<std::string> & factor_owners)
{
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
        auto edge_key = std::make_pair(std::min(keys[a], keys[b]), std::max(keys[a], keys[b]));
        if (!owner.empty()) edge_owners_[edge_key] = owner;
      }
    }
  }
}

const std::unordered_map<gtsam::Key, std::vector<gtsam::Key>> & MapManager::getAdjacency() const
{
  return adjacency_;
}

std::string MapManager::getEdgeOwner(gtsam::Key key_a, gtsam::Key key_b) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto edge_key = std::make_pair(std::min(key_a, key_b), std::max(key_a, key_b));
  auto it = edge_owners_.find(edge_key);
  return (it != edge_owners_.end()) ? it->second : "";
}

// ==========================================================================
// SQLite Persistence
// ==========================================================================

static void execSql(sqlite3 * db, const char * sql)
{
  char * err = nullptr;
  sqlite3_exec(db, sql, nullptr, nullptr, &err);
  if (err) sqlite3_free(err);
}

bool MapManager::saveMap(const std::string & path)
{
  std::lock_guard<std::mutex> lock(mtx_);
  namespace fs = std::filesystem;

  // Ensure parent directory exists
  fs::create_directories(fs::path(path).parent_path());

  sqlite3 * db = nullptr;
  if (sqlite3_open(path.c_str(), &db) != SQLITE_OK) return false;

  // Create tables
  execSql(db, "PRAGMA journal_mode=WAL;");
  execSql(db, "BEGIN TRANSACTION;");

  execSql(db, "CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT);");
  execSql(
    db,
    "CREATE TABLE IF NOT EXISTS keyframes ("
    "id INTEGER PRIMARY KEY, gtsam_key INTEGER UNIQUE, "
    "x REAL, y REAL, z REAL, roll REAL, pitch REAL, yaw REAL, "
    "time REAL, owner TEXT);");
  execSql(
    db,
    "CREATE TABLE IF NOT EXISTS keyframe_data ("
    "gtsam_key INTEGER, data_key TEXT, data BLOB, "
    "PRIMARY KEY (gtsam_key, data_key));");
  execSql(
    db,
    "CREATE TABLE IF NOT EXISTS global_data ("
    "data_key TEXT PRIMARY KEY, data BLOB);");
  execSql(
    db,
    "CREATE TABLE IF NOT EXISTS edges ("
    "key_a INTEGER, key_b INTEGER, owner TEXT, "
    "PRIMARY KEY (key_a, key_b));");
  execSql(
    db,
    "CREATE TABLE IF NOT EXISTS data_formats ("
    "data_key TEXT PRIMARY KEY, format TEXT, scope TEXT);");

  // Metadata
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(db, "INSERT INTO metadata VALUES(?,?)", -1, &stmt, nullptr);
    auto insert = [&](const char * k, const std::string & v) {
      sqlite3_bind_text(stmt, 1, k, -1, SQLITE_STATIC);
      sqlite3_bind_text(stmt, 2, v.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_step(stmt);
      sqlite3_reset(stmt);
    };
    insert("version", "4");
    insert("num_states", std::to_string(key_list_.size()));
    sqlite3_finalize(stmt);
  }

  // Keyframe poses
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(db, "INSERT INTO keyframes VALUES(?,?,?,?,?,?,?,?,?,?)", -1, &stmt, nullptr);
    for (size_t i = 0; i < key_list_.size(); i++) {
      auto k = key_list_[i];
      auto & p = key_poses_6d_->points[i];
      std::string owner;
      auto oit = key_owner_plugin_.find(k);
      if (oit != key_owner_plugin_.end()) owner = oit->second;

      sqlite3_bind_int(stmt, 1, static_cast<int>(i));
      sqlite3_bind_int64(stmt, 2, static_cast<int64_t>(k));
      sqlite3_bind_double(stmt, 3, p.x);
      sqlite3_bind_double(stmt, 4, p.y);
      sqlite3_bind_double(stmt, 5, p.z);
      sqlite3_bind_double(stmt, 6, p.roll);
      sqlite3_bind_double(stmt, 7, p.pitch);
      sqlite3_bind_double(stmt, 8, p.yaw);
      sqlite3_bind_double(stmt, 9, p.time);
      sqlite3_bind_text(stmt, 10, owner.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_step(stmt);
      sqlite3_reset(stmt);
    }
    sqlite3_finalize(stmt);
  }

  const auto & fmt_reg = formats::registry();

  // Keyframe data blobs
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(db, "INSERT INTO keyframe_data VALUES(?,?,?)", -1, &stmt, nullptr);

    for (const auto & [data_key, format_name] : keyframe_formats_) {
      auto fit = fmt_reg.find(format_name);
      if (fit == fmt_reg.end()) continue;

      for (size_t i = 0; i < key_list_.size(); i++) {
        auto kit = keyframe_data_.find(key_list_[i]);
        if (kit == keyframe_data_.end()) continue;
        auto dit = kit->second.find(data_key);
        if (dit == kit->second.end()) continue;

        try {
          auto bytes = fit->second->serialize(dit->second);
          if (bytes.empty()) continue;

          sqlite3_bind_int64(stmt, 1, static_cast<int64_t>(key_list_[i]));
          sqlite3_bind_text(stmt, 2, data_key.c_str(), -1, SQLITE_TRANSIENT);
          sqlite3_bind_blob(stmt, 3, bytes.data(), static_cast<int>(bytes.size()), SQLITE_TRANSIENT);
          sqlite3_step(stmt);
          sqlite3_reset(stmt);
        } catch (const std::bad_any_cast & e) {
          // Type contract violation — log and skip
        }
      }
    }
    sqlite3_finalize(stmt);
  }

  // Global data blobs
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(db, "INSERT INTO global_data VALUES(?,?)", -1, &stmt, nullptr);

    for (const auto & [data_key, format_name] : global_formats_) {
      auto fit = fmt_reg.find(format_name);
      if (fit == fmt_reg.end()) continue;
      auto dit = global_data_.find(data_key);
      if (dit == global_data_.end()) continue;

      try {
        auto bytes = fit->second->serialize(dit->second);
        if (bytes.empty()) continue;

        sqlite3_bind_text(stmt, 1, data_key.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_blob(stmt, 2, bytes.data(), static_cast<int>(bytes.size()), SQLITE_TRANSIENT);
        sqlite3_step(stmt);
        sqlite3_reset(stmt);
      } catch (const std::bad_any_cast &) {
      }
    }
    sqlite3_finalize(stmt);
  }

  // Edges
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(db, "INSERT INTO edges VALUES(?,?,?)", -1, &stmt, nullptr);
    for (const auto & [edge, owner] : edge_owners_) {
      sqlite3_bind_int64(stmt, 1, static_cast<int64_t>(edge.first));
      sqlite3_bind_int64(stmt, 2, static_cast<int64_t>(edge.second));
      sqlite3_bind_text(stmt, 3, owner.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_step(stmt);
      sqlite3_reset(stmt);
    }
    sqlite3_finalize(stmt);
  }

  // Data formats (self-describing)
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(db, "INSERT INTO data_formats VALUES(?,?,?)", -1, &stmt, nullptr);
    for (const auto & [dk, fmt] : keyframe_formats_) {
      sqlite3_bind_text(stmt, 1, dk.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 2, fmt.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 3, "keyframe", -1, SQLITE_STATIC);
      sqlite3_step(stmt);
      sqlite3_reset(stmt);
    }
    for (const auto & [dk, fmt] : global_formats_) {
      sqlite3_bind_text(stmt, 1, dk.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 2, fmt.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 3, "global", -1, SQLITE_STATIC);
      sqlite3_step(stmt);
      sqlite3_reset(stmt);
    }
    sqlite3_finalize(stmt);
  }

  execSql(db, "COMMIT;");
  sqlite3_close(db);
  return true;
}

bool MapManager::loadMap(const std::string & path)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!std::filesystem::exists(path)) return false;

  sqlite3 * db = nullptr;
  if (sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) return false;

  // Clear existing state
  key_poses_3d_->clear();
  key_poses_6d_->clear();
  keyframe_data_.clear();
  key_to_cloud_index_.clear();
  key_list_.clear();
  key_owner_plugin_.clear();
  global_data_.clear();
  adjacency_.clear();
  edge_owners_.clear();

  // Load keyframe poses
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(
      db,
      "SELECT id, gtsam_key, x, y, z, roll, pitch, yaw, time, owner "
      "FROM keyframes ORDER BY id",
      -1,
      &stmt,
      nullptr);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
      int idx = sqlite3_column_int(stmt, 0);
      auto gtsam_key = static_cast<gtsam::Key>(sqlite3_column_int64(stmt, 1));

      PoseType p;
      p.x = static_cast<float>(sqlite3_column_double(stmt, 2));
      p.y = static_cast<float>(sqlite3_column_double(stmt, 3));
      p.z = static_cast<float>(sqlite3_column_double(stmt, 4));
      p.roll = static_cast<float>(sqlite3_column_double(stmt, 5));
      p.pitch = static_cast<float>(sqlite3_column_double(stmt, 6));
      p.yaw = static_cast<float>(sqlite3_column_double(stmt, 7));
      p.time = sqlite3_column_double(stmt, 8);
      p.intensity = static_cast<float>(idx);

      PointType p3d;
      p3d.x = p.x;
      p3d.y = p.y;
      p3d.z = p.z;
      p3d.intensity = static_cast<float>(idx);

      key_poses_3d_->push_back(p3d);
      key_poses_6d_->push_back(p);
      key_to_cloud_index_[gtsam_key] = idx;
      key_list_.push_back(gtsam_key);

      auto owner_col = sqlite3_column_text(stmt, 9);
      if (owner_col) {
        std::string owner(reinterpret_cast<const char *>(owner_col));
        if (!owner.empty()) key_owner_plugin_[gtsam_key] = owner;
      }
    }
    sqlite3_finalize(stmt);
  }

  // Load data formats
  std::unordered_map<std::string, std::string> loaded_formats;  // data_key → format
  std::unordered_map<std::string, std::string> loaded_scopes;  // data_key → scope
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(db, "SELECT data_key, format, scope FROM data_formats", -1, &stmt, nullptr);
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      std::string dk(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
      std::string fmt(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)));
      std::string scope(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2)));
      loaded_formats[dk] = fmt;
      loaded_scopes[dk] = scope;
    }
    sqlite3_finalize(stmt);
  }

  const auto & fmt_reg = formats::registry();

  // Load keyframe data blobs
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(db, "SELECT gtsam_key, data_key, data FROM keyframe_data", -1, &stmt, nullptr);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
      auto gtsam_key = static_cast<gtsam::Key>(sqlite3_column_int64(stmt, 0));
      std::string data_key(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)));

      auto fmt_it = loaded_formats.find(data_key);
      if (fmt_it == loaded_formats.end()) continue;
      auto reg_it = fmt_reg.find(fmt_it->second);
      if (reg_it == fmt_reg.end()) continue;

      const void * blob = sqlite3_column_blob(stmt, 2);
      int blob_size = sqlite3_column_bytes(stmt, 2);
      if (!blob || blob_size <= 0) continue;

      std::vector<uint8_t> bytes(static_cast<const uint8_t *>(blob), static_cast<const uint8_t *>(blob) + blob_size);
      try {
        auto data = reg_it->second->deserialize(bytes);
        if (data.has_value()) {
          keyframe_data_[gtsam_key][data_key] = std::move(data);
        }
      } catch (...) {
      }
    }
    sqlite3_finalize(stmt);
  }

  // Load global data
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(db, "SELECT data_key, data FROM global_data", -1, &stmt, nullptr);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
      std::string data_key(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));

      auto fmt_it = loaded_formats.find(data_key);
      if (fmt_it == loaded_formats.end()) continue;
      auto reg_it = fmt_reg.find(fmt_it->second);
      if (reg_it == fmt_reg.end()) continue;

      const void * blob = sqlite3_column_blob(stmt, 1);
      int blob_size = sqlite3_column_bytes(stmt, 1);
      if (!blob || blob_size <= 0) continue;

      std::vector<uint8_t> bytes(static_cast<const uint8_t *>(blob), static_cast<const uint8_t *>(blob) + blob_size);
      try {
        auto data = reg_it->second->deserialize(bytes);
        if (data.has_value()) {
          global_data_[data_key] = std::move(data);
        }
      } catch (...) {
      }
    }
    sqlite3_finalize(stmt);
  }

  // Load edges
  {
    sqlite3_stmt * stmt;
    sqlite3_prepare_v2(db, "SELECT key_a, key_b, owner FROM edges", -1, &stmt, nullptr);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
      auto ka = static_cast<gtsam::Key>(sqlite3_column_int64(stmt, 0));
      auto kb = static_cast<gtsam::Key>(sqlite3_column_int64(stmt, 1));
      auto owner_col = sqlite3_column_text(stmt, 2);
      std::string owner = owner_col ? reinterpret_cast<const char *>(owner_col) : "";

      adjacency_[ka].push_back(kb);
      adjacency_[kb].push_back(ka);
      if (!owner.empty()) {
        edge_owners_[std::make_pair(std::min(ka, kb), std::max(ka, kb))] = owner;
      }
    }
    sqlite3_finalize(stmt);
  }

  // Track prior map keys
  prior_map_keys_.clear();
  for (auto k : key_list_) prior_map_keys_.insert(k);

  kdtree_dirty_ = true;
  prior_map_loaded_ = true;

  sqlite3_close(db);
  return true;
}

bool MapManager::isPriorMapKey(gtsam::Key key) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return prior_map_keys_.count(key) > 0;
}

bool MapManager::hasPriorMap() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return prior_map_loaded_;
}

}  // namespace eidos
