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

#include "eidos/map/map_manager.hpp"

#include <gtsam/inference/Symbol.h>
#include <sqlite3.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "eidos/map/registry.hpp"

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
  if (owner.empty()) {
    RCLCPP_ERROR(logger_, "\033[33m[MapManager]\033[0m addKeyframe called with empty owner for key %lu", gtsam_key);
    return;
  }
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
  std::lock_guard<std::mutex> lock(mtx_);
  return pcl::make_shared<pcl::PointCloud<PointType>>(*key_poses_3d_);
}

pcl::PointCloud<PoseType>::Ptr MapManager::getKeyPoses6D() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return pcl::make_shared<pcl::PointCloud<PoseType>>(*key_poses_6d_);
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

std::unordered_map<gtsam::Key, std::vector<gtsam::Key>> MapManager::getAdjacency() const
{
  std::lock_guard<std::mutex> lock(mtx_);
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

static bool execSql(sqlite3 * db, const char * sql)
{
  char * err = nullptr;
  int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  if (err) sqlite3_free(err);
  return rc == SQLITE_OK;
}

bool MapManager::saveMap(const std::string & path)
{
  namespace fs = std::filesystem;

  // Snapshot everything under the lock; SQLite I/O happens after releasing it.
  struct KeyframeRow
  {
    gtsam::Key key;
    std::array<double, 7> pose;  // x, y, z, roll, pitch, yaw, time
    std::string owner;
  };
  std::vector<KeyframeRow> keyframes;
  std::vector<std::tuple<gtsam::Key, std::string, std::vector<uint8_t>>> keyframe_blobs;
  std::vector<std::pair<std::string, std::vector<uint8_t>>> global_blobs;
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::string> edges;
  std::vector<std::tuple<std::string, std::string, const char *>> data_formats;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const auto & fmt_reg = formats::registry();

    for (size_t i = 0; i < key_list_.size(); i++) {
      const auto & p = key_poses_6d_->points[i];
      auto oit = key_owner_plugin_.find(key_list_[i]);
      keyframes.push_back(
        {key_list_[i],
         {p.x, p.y, p.z, p.roll, p.pitch, p.yaw, p.time},
         oit != key_owner_plugin_.end() ? oit->second : ""});
    }

    for (const auto & [data_key, format_name] : keyframe_formats_) {
      auto fit = fmt_reg.find(format_name);
      if (fit == fmt_reg.end()) continue;
      for (auto k : key_list_) {
        auto kit = keyframe_data_.find(k);
        if (kit == keyframe_data_.end()) continue;
        auto dit = kit->second.find(data_key);
        if (dit == kit->second.end()) continue;
        try {
          auto bytes = fit->second->serialize(dit->second);
          if (!bytes.empty()) keyframe_blobs.emplace_back(k, data_key, std::move(bytes));
        } catch (const std::bad_any_cast &) {
          RCLCPP_WARN(logger_, "\033[33m[MapManager]\033[0m Type mismatch serializing '%s', skipped", data_key.c_str());
        }
      }
    }

    for (const auto & [data_key, format_name] : global_formats_) {
      auto fit = fmt_reg.find(format_name);
      if (fit == fmt_reg.end()) continue;
      auto dit = global_data_.find(data_key);
      if (dit == global_data_.end()) continue;
      try {
        auto bytes = fit->second->serialize(dit->second);
        if (!bytes.empty()) global_blobs.emplace_back(data_key, std::move(bytes));
      } catch (const std::bad_any_cast &) {
        RCLCPP_WARN(logger_, "\033[33m[MapManager]\033[0m Type mismatch serializing '%s', skipped", data_key.c_str());
      }
    }

    for (const auto & [a, neighbors] : adjacency_) {
      for (auto b : neighbors) edges.emplace(std::make_pair(std::min(a, b), std::max(a, b)), "");
    }
    for (const auto & [edge, owner] : edge_owners_) edges[edge] = owner;

    for (const auto & [dk, fmt] : keyframe_formats_) data_formats.emplace_back(dk, fmt, "keyframe");
    for (const auto & [dk, fmt] : global_formats_) data_formats.emplace_back(dk, fmt, "global");
  }

  // Write a fresh database to a temp file, then atomically rename it over the target.
  std::error_code ec;
  auto parent = fs::path(path).parent_path();
  if (!parent.empty()) fs::create_directories(parent, ec);
  const std::string tmp_path = path + ".tmp";
  fs::remove(tmp_path, ec);

  sqlite3 * db = nullptr;
  sqlite3_stmt * stmt = nullptr;
  auto fail = [&](const char * what) {
    RCLCPP_ERROR(
      logger_,
      "\033[33m[MapManager]\033[0m Failed to save map %s: %s (%s)",
      path.c_str(),
      what,
      db ? sqlite3_errmsg(db) : "no db");
    sqlite3_finalize(stmt);
    if (db) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      sqlite3_close(db);
    }
    fs::remove(tmp_path, ec);
    return false;
  };
  auto prepare = [&](const char * sql) {
    sqlite3_finalize(stmt);
    stmt = nullptr;
    return sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) == SQLITE_OK;
  };
  auto step = [&]() {
    int rc = sqlite3_step(stmt);
    sqlite3_reset(stmt);
    return rc == SQLITE_DONE;
  };

  if (sqlite3_open(tmp_path.c_str(), &db) != SQLITE_OK) return fail("open");
  if (!execSql(db, "BEGIN TRANSACTION;")) return fail("begin");
  if (
    !execSql(db, "CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT);") ||
    !execSql(
      db,
      "CREATE TABLE keyframes ("
      "id INTEGER PRIMARY KEY, gtsam_key INTEGER UNIQUE, "
      "x REAL, y REAL, z REAL, roll REAL, pitch REAL, yaw REAL, "
      "time REAL, owner TEXT);") ||
    !execSql(
      db,
      "CREATE TABLE keyframe_data ("
      "gtsam_key INTEGER, data_key TEXT, data BLOB, "
      "PRIMARY KEY (gtsam_key, data_key));") ||
    !execSql(db, "CREATE TABLE global_data (data_key TEXT PRIMARY KEY, data BLOB);") ||
    !execSql(
      db,
      "CREATE TABLE edges ("
      "key_a INTEGER, key_b INTEGER, owner TEXT, "
      "PRIMARY KEY (key_a, key_b));") ||
    !execSql(db, "CREATE TABLE data_formats (data_key TEXT PRIMARY KEY, format TEXT, scope TEXT);"))
  {
    return fail("create tables");
  }

  // Metadata
  if (!prepare("INSERT INTO metadata VALUES(?,?)")) return fail("prepare metadata");
  auto insert_metadata = [&](const char * k, const std::string & v) {
    sqlite3_bind_text(stmt, 1, k, -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, v.c_str(), -1, SQLITE_TRANSIENT);
    return step();
  };
  if (!insert_metadata("version", "4") || !insert_metadata("num_states", std::to_string(keyframes.size()))) {
    return fail("insert metadata");
  }

  // Keyframe poses
  if (!prepare("INSERT INTO keyframes VALUES(?,?,?,?,?,?,?,?,?,?)")) return fail("prepare keyframes");
  for (size_t i = 0; i < keyframes.size(); i++) {
    const auto & kf = keyframes[i];
    sqlite3_bind_int(stmt, 1, static_cast<int>(i));
    sqlite3_bind_int64(stmt, 2, static_cast<int64_t>(kf.key));
    for (int j = 0; j < 7; j++) sqlite3_bind_double(stmt, 3 + j, kf.pose[j]);
    sqlite3_bind_text(stmt, 10, kf.owner.c_str(), -1, SQLITE_TRANSIENT);
    if (!step()) return fail("insert keyframe");
  }

  // Keyframe data blobs
  if (!prepare("INSERT INTO keyframe_data VALUES(?,?,?)")) return fail("prepare keyframe_data");
  for (const auto & [k, data_key, bytes] : keyframe_blobs) {
    sqlite3_bind_int64(stmt, 1, static_cast<int64_t>(k));
    sqlite3_bind_text(stmt, 2, data_key.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_bind_blob64(stmt, 3, bytes.data(), bytes.size(), SQLITE_STATIC) != SQLITE_OK || !step()) {
      return fail("insert keyframe_data");
    }
  }

  // Global data blobs
  if (!prepare("INSERT INTO global_data VALUES(?,?)")) return fail("prepare global_data");
  for (const auto & [data_key, bytes] : global_blobs) {
    sqlite3_bind_text(stmt, 1, data_key.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_bind_blob64(stmt, 2, bytes.data(), bytes.size(), SQLITE_STATIC) != SQLITE_OK || !step()) {
      return fail("insert global_data");
    }
  }

  // Edges
  if (!prepare("INSERT INTO edges VALUES(?,?,?)")) return fail("prepare edges");
  for (const auto & [edge, owner] : edges) {
    sqlite3_bind_int64(stmt, 1, static_cast<int64_t>(edge.first));
    sqlite3_bind_int64(stmt, 2, static_cast<int64_t>(edge.second));
    sqlite3_bind_text(stmt, 3, owner.c_str(), -1, SQLITE_TRANSIENT);
    if (!step()) return fail("insert edge");
  }

  // Data formats (self-describing)
  if (!prepare("INSERT INTO data_formats VALUES(?,?,?)")) return fail("prepare data_formats");
  for (const auto & [dk, fmt, scope] : data_formats) {
    sqlite3_bind_text(stmt, 1, dk.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, fmt.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, scope, -1, SQLITE_STATIC);
    if (!step()) return fail("insert data_format");
  }

  sqlite3_finalize(stmt);
  stmt = nullptr;
  if (!execSql(db, "COMMIT;")) return fail("commit");
  if (sqlite3_close(db) != SQLITE_OK) {
    db = nullptr;
    return fail("close");
  }
  db = nullptr;

  // Stale WAL/SHM files from older saves would otherwise be replayed onto the new file.
  fs::remove(path + "-wal", ec);
  fs::remove(path + "-shm", ec);
  fs::rename(tmp_path, path, ec);
  if (ec) {
    RCLCPP_ERROR(
      logger_,
      "\033[33m[MapManager]\033[0m Failed to move %s to %s: %s",
      tmp_path.c_str(),
      path.c_str(),
      ec.message().c_str());
    fs::remove(tmp_path, ec);
    return false;
  }

  RCLCPP_INFO(
    logger_,
    "\033[33m[MapManager]\033[0m Saved map: %s (%zu keyframes, %zu edges)",
    path.c_str(),
    keyframes.size(),
    edges.size());
  return true;
}

bool MapManager::loadMap(const std::string & path)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!std::filesystem::exists(path)) {
    RCLCPP_ERROR(logger_, "\033[33m[MapManager]\033[0m Map file not found: %s", path.c_str());
    return false;
  }

  sqlite3 * db = nullptr;
  if (sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "\033[33m[MapManager]\033[0m Failed to open map file for reading: %s", path.c_str());
    return false;
  }

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
  RCLCPP_INFO(
    logger_,
    "\033[33m[MapManager]\033[0m Loaded map: %s (%zu keyframes, %zu edges)",
    path.c_str(),
    key_list_.size(),
    edge_owners_.size());
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
