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

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <sqlite3.h>

#include <algorithm>
#include <any>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <small_gicp/points/point_cloud.hpp>

#include "eidos/formats/registry.hpp"
#include "eidos/utils/types.hpp"

// ============================================================================
// Helpers
// ============================================================================

using PointType = eidos::PointType;

struct sqlite3_deleter
{
  void operator()(sqlite3 * db) const
  {
    if (db) sqlite3_close(db);
  }
};

using SqlitePtr = std::unique_ptr<sqlite3, sqlite3_deleter>;

static SqlitePtr openMap(const std::string & path)
{
  if (!std::filesystem::exists(path)) {
    std::cerr << "Error: file not found: " << path << "\n";
    return nullptr;
  }
  sqlite3 * raw = nullptr;
  if (sqlite3_open_v2(path.c_str(), &raw, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    std::cerr << "Error: failed to open SQLite database: " << path << "\n";
    return nullptr;
  }
  return SqlitePtr(raw);
}

struct stmt_deleter
{
  void operator()(sqlite3_stmt * s) const
  {
    if (s) sqlite3_finalize(s);
  }
};

using StmtPtr = std::unique_ptr<sqlite3_stmt, stmt_deleter>;

static StmtPtr prepare(sqlite3 * db, const char * sql)
{
  sqlite3_stmt * raw = nullptr;
  sqlite3_prepare_v2(db, sql, -1, &raw, nullptr);
  return StmtPtr(raw);
}

static void printUsage()
{
  std::cout << "Usage: eidos <command> [options]\n\n"
            << "Commands:\n"
            << "  info  <map_file>                                   Show map metadata\n"
            << "  poses <map_file>                                   Dump keyframe poses as CSV\n"
            << "  map   <map_file> --key <data_key> -o <output.pcd>  Build combined PCD\n"
            << "  graph <map_file>                                   Print factor graph edges\n";
}

// ============================================================================
// info
// ============================================================================

static int cmdInfo(const std::string & path)
{
  auto db = openMap(path);
  if (!db) return 1;

  std::cout << "Map: " << path << "\n";

  // Metadata
  {
    auto stmt = prepare(db.get(), "SELECT key, value FROM metadata");
    while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      auto key = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 0));
      auto val = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 1));
      if (key && val) {
        std::string k(key);
        if (k == "version") std::cout << "Version: " << val << "\n";
      }
    }
  }

  // Keyframe count + bounds
  {
    auto stmt = prepare(db.get(), "SELECT COUNT(*), MIN(x), MAX(x), MIN(y), MAX(y), MIN(z), MAX(z) FROM keyframes");
    if (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      int count = sqlite3_column_int(stmt.get(), 0);
      std::cout << "Keyframes: " << count << "\n";
      if (count > 0) {
        std::cout << std::fixed << std::setprecision(1) << "Bounds: x=[" << sqlite3_column_double(stmt.get(), 1) << ", "
                  << sqlite3_column_double(stmt.get(), 2) << "]  y=[" << sqlite3_column_double(stmt.get(), 3) << ", "
                  << sqlite3_column_double(stmt.get(), 4) << "]  z=[" << sqlite3_column_double(stmt.get(), 5) << ", "
                  << sqlite3_column_double(stmt.get(), 6) << "]\n";
      }
    }
  }

  // Edge count
  {
    auto stmt = prepare(db.get(), "SELECT COUNT(*) FROM edges");
    if (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      std::cout << "Edges: " << sqlite3_column_int(stmt.get(), 0) << "\n";
    }
  }

  // Data formats
  {
    auto stmt = prepare(db.get(), "SELECT data_key, format, scope FROM data_formats ORDER BY scope, data_key");
    std::cout << "\nData formats:\n";
    while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      auto dk = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 0));
      auto fmt = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 1));
      auto scope = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 2));
      if (dk && fmt && scope) {
        std::cout << "  " << std::left << std::setw(28) << dk << std::setw(20) << fmt << scope << "\n";
      }
    }
  }

  // Global data (decoded)
  const auto & reg = eidos::formats::registry();
  {
    // First load global format mappings
    std::unordered_map<std::string, std::string> global_fmts;
    {
      auto stmt = prepare(db.get(), "SELECT data_key, format FROM data_formats WHERE scope='global'");
      while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
        auto dk = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 0));
        auto fmt = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 1));
        if (dk && fmt) global_fmts[dk] = fmt;
      }
    }

    auto stmt = prepare(db.get(), "SELECT data_key, data FROM global_data");
    bool header_printed = false;
    while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      auto dk = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 0));
      if (!dk) continue;
      std::string data_key(dk);

      if (!header_printed) {
        std::cout << "\nGlobal data:\n";
        header_printed = true;
      }

      auto fmt_it = global_fmts.find(data_key);
      if (fmt_it == global_fmts.end()) {
        int sz = sqlite3_column_bytes(stmt.get(), 1);
        std::cout << "  " << data_key << ": (" << sz << " bytes, unknown format)\n";
        continue;
      }

      auto reg_it = reg.find(fmt_it->second);
      if (reg_it == reg.end()) {
        int sz = sqlite3_column_bytes(stmt.get(), 1);
        std::cout << "  " << data_key << ": (" << sz << " bytes, format '" << fmt_it->second << "' not in registry)\n";
        continue;
      }

      const void * blob = sqlite3_column_blob(stmt.get(), 1);
      int blob_size = sqlite3_column_bytes(stmt.get(), 1);
      if (!blob || blob_size <= 0) continue;

      std::vector<uint8_t> bytes(static_cast<const uint8_t *>(blob), static_cast<const uint8_t *>(blob) + blob_size);

      try {
        auto data = reg_it->second->deserialize(bytes);

        // Print based on format type
        if (fmt_it->second == "raw_double3") {
          auto pt = std::any_cast<gtsam::Point3>(data);
          std::cout << std::fixed << std::setprecision(6) << "  " << data_key << ": [" << pt.x() << ", " << pt.y()
                    << ", " << pt.z() << "]\n";
        } else if (fmt_it->second == "raw_double5") {
          auto arr = std::any_cast<std::array<double, 5>>(data);
          std::cout << std::fixed << std::setprecision(6) << "  " << data_key << ": [" << arr[0] << ", " << arr[1]
                    << ", " << arr[2] << ", " << arr[3] << ", " << arr[4] << "]\n";
        } else if (fmt_it->second == "raw_double4_eigen") {
          auto v = std::any_cast<Eigen::Vector4d>(data);
          std::cout << std::fixed << std::setprecision(6) << "  " << data_key << ": [" << v[0] << ", " << v[1] << ", "
                    << v[2] << ", " << v[3] << "]\n";
        } else {
          std::cout << "  " << data_key << ": (" << blob_size << " bytes, format '" << fmt_it->second << "')\n";
        }
      } catch (const std::exception & e) {
        std::cout << "  " << data_key << ": (decode error: " << e.what() << ")\n";
      }
    }
  }

  return 0;
}

// ============================================================================
// poses
// ============================================================================

static int cmdPoses(const std::string & path)
{
  auto db = openMap(path);
  if (!db) return 1;

  std::cout << "id,gtsam_key,x,y,z,roll,pitch,yaw,time,owner\n";

  auto stmt = prepare(
    db.get(),
    "SELECT id, gtsam_key, x, y, z, roll, pitch, yaw, time, owner "
    "FROM keyframes ORDER BY id");

  while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
    int id = sqlite3_column_int(stmt.get(), 0);
    int64_t gk = sqlite3_column_int64(stmt.get(), 1);
    double x = sqlite3_column_double(stmt.get(), 2);
    double y = sqlite3_column_double(stmt.get(), 3);
    double z = sqlite3_column_double(stmt.get(), 4);
    double roll = sqlite3_column_double(stmt.get(), 5);
    double pitch = sqlite3_column_double(stmt.get(), 6);
    double yaw = sqlite3_column_double(stmt.get(), 7);
    double time = sqlite3_column_double(stmt.get(), 8);
    auto owner_col = sqlite3_column_text(stmt.get(), 9);
    std::string owner = owner_col ? reinterpret_cast<const char *>(owner_col) : "";

    std::cout << std::fixed << std::setprecision(6) << id << "," << gk << "," << x << "," << y << "," << z << ","
              << roll << "," << pitch << "," << yaw << "," << std::setprecision(3) << time << "," << owner << "\n";
  }

  return 0;
}

// ============================================================================
// map
// ============================================================================

struct MapArgs
{
  std::string map_path;
  std::string data_key;
  std::string output_path;
  double voxel_size = 0.0;
  int skip = 1;
};

static bool parseMapArgs(int argc, char ** argv, MapArgs & args)
{
  // argv layout: eidos map <map_file> --key <key> -o <out> [--voxel <size>] [--skip <N>]
  if (argc < 3) return false;
  args.map_path = argv[2];

  for (int i = 3; i < argc; i++) {
    std::string arg(argv[i]);
    if (arg == "--key" && i + 1 < argc) {
      args.data_key = argv[++i];
    } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
      args.output_path = argv[++i];
    } else if (arg == "--voxel" && i + 1 < argc) {
      args.voxel_size = std::stod(argv[++i]);
    } else if (arg == "--skip" && i + 1 < argc) {
      args.skip = std::stoi(argv[++i]);
    }
  }

  if (args.data_key.empty()) {
    std::cerr << "Error: --key <data_key> is required\n";
    return false;
  }
  if (args.output_path.empty()) {
    std::cerr << "Error: -o <output.pcd> is required\n";
    return false;
  }
  if (args.skip < 1) args.skip = 1;
  return true;
}

static Eigen::Affine3f poseToTransform(float x, float y, float z, float roll, float pitch, float yaw)
{
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  t.translation() = Eigen::Vector3f(x, y, z);
  t.linear() = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()))
                 .toRotationMatrix();
  return t;
}

static int cmdMap(int argc, char ** argv)
{
  MapArgs args;
  if (!parseMapArgs(argc, argv, args)) {
    std::cerr << "Usage: eidos map <map_file> --key <data_key> -o <output.pcd> [--voxel <size>] [--skip <N>]\n";
    return 1;
  }

  auto db = openMap(args.map_path);
  if (!db) return 1;

  // Resolve format for the requested data_key
  std::string format_name;
  {
    auto stmt = prepare(db.get(), "SELECT format FROM data_formats WHERE data_key=? AND scope='keyframe'");
    sqlite3_bind_text(stmt.get(), 1, args.data_key.c_str(), -1, SQLITE_STATIC);
    if (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      format_name = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 0));
    }
  }

  if (format_name.empty()) {
    std::cerr << "Error: data_key '" << args.data_key << "' not found in data_formats (keyframe scope)\n";
    std::cerr << "Available keyframe data keys:\n";
    auto stmt = prepare(db.get(), "SELECT data_key, format FROM data_formats WHERE scope='keyframe'");
    while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      auto dk = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 0));
      auto fmt = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 1));
      if (dk && fmt) std::cerr << "  " << dk << " (" << fmt << ")\n";
    }
    return 1;
  }

  const auto & reg = eidos::formats::registry();
  auto reg_it = reg.find(format_name);
  if (reg_it == reg.end()) {
    std::cerr << "Error: format '" << format_name << "' not found in registry\n";
    return 1;
  }

  bool is_pcl = (format_name == "pcl_pcd_binary");
  bool is_gicp = (format_name == "small_gicp_binary");
  if (!is_pcl && !is_gicp) {
    std::cerr << "Error: format '" << format_name << "' is not a point cloud format (expected "
              << "pcl_pcd_binary or small_gicp_binary)\n";
    return 1;
  }

  // Load keyframe poses indexed by gtsam_key
  struct KfPose
  {
    int id;
    float x, y, z, roll, pitch, yaw;
  };

  std::unordered_map<int64_t, KfPose> poses;
  {
    auto stmt = prepare(db.get(), "SELECT id, gtsam_key, x, y, z, roll, pitch, yaw FROM keyframes ORDER BY id");
    while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      KfPose p;
      p.id = sqlite3_column_int(stmt.get(), 0);
      int64_t gk = sqlite3_column_int64(stmt.get(), 1);
      p.x = static_cast<float>(sqlite3_column_double(stmt.get(), 2));
      p.y = static_cast<float>(sqlite3_column_double(stmt.get(), 3));
      p.z = static_cast<float>(sqlite3_column_double(stmt.get(), 4));
      p.roll = static_cast<float>(sqlite3_column_double(stmt.get(), 5));
      p.pitch = static_cast<float>(sqlite3_column_double(stmt.get(), 6));
      p.yaw = static_cast<float>(sqlite3_column_double(stmt.get(), 7));
      poses[gk] = p;
    }
  }

  // Load and transform clouds
  auto merged = pcl::make_shared<pcl::PointCloud<PointType>>();
  int processed = 0;
  int skipped = 0;

  {
    auto stmt = prepare(db.get(), "SELECT gtsam_key, data FROM keyframe_data WHERE data_key=?");
    sqlite3_bind_text(stmt.get(), 1, args.data_key.c_str(), -1, SQLITE_STATIC);

    while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      int64_t gk = sqlite3_column_int64(stmt.get(), 0);

      auto pose_it = poses.find(gk);
      if (pose_it == poses.end()) continue;

      // Apply skip
      if (pose_it->second.id % args.skip != 0) {
        skipped++;
        continue;
      }

      const void * blob = sqlite3_column_blob(stmt.get(), 1);
      int blob_size = sqlite3_column_bytes(stmt.get(), 1);
      if (!blob || blob_size <= 0) continue;

      std::vector<uint8_t> bytes(static_cast<const uint8_t *>(blob), static_cast<const uint8_t *>(blob) + blob_size);

      try {
        auto data = reg_it->second->deserialize(bytes);
        pcl::PointCloud<PointType>::Ptr cloud;

        if (is_pcl) {
          cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
        } else {
          // Convert small_gicp to PCL
          auto gicp_cloud = std::any_cast<small_gicp::PointCloud::Ptr>(data);
          cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
          cloud->resize(gicp_cloud->size());
          for (size_t i = 0; i < gicp_cloud->size(); i++) {
            const auto & pt = gicp_cloud->point(i);
            cloud->points[i].x = static_cast<float>(pt.x());
            cloud->points[i].y = static_cast<float>(pt.y());
            cloud->points[i].z = static_cast<float>(pt.z());
            cloud->points[i].intensity = 0.0f;
          }
        }

        if (cloud && !cloud->empty()) {
          auto & p = pose_it->second;
          auto tf = poseToTransform(p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
          pcl::PointCloud<PointType> transformed;
          pcl::transformPointCloud(*cloud, transformed, tf);
          *merged += transformed;
          processed++;
        }
      } catch (const std::exception & e) {
        std::cerr << "Warning: failed to deserialize keyframe " << pose_it->second.id << ": " << e.what() << "\n";
      }
    }
  }

  std::cerr << "Processed " << processed << " keyframes";
  if (skipped > 0) std::cerr << " (skipped " << skipped << ")";
  std::cerr << ", " << merged->size() << " points";

  // Voxel downsample
  if (args.voxel_size > 0.0 && !merged->empty()) {
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(merged);
    float vs = static_cast<float>(args.voxel_size);
    vg.setLeafSize(vs, vs, vs);
    auto filtered = pcl::make_shared<pcl::PointCloud<PointType>>();
    vg.filter(*filtered);
    std::cerr << " -> " << filtered->size() << " after voxel(" << args.voxel_size << ")";
    merged = filtered;
  }

  std::cerr << "\n";

  if (merged->empty()) {
    std::cerr << "Warning: resulting cloud is empty\n";
    return 1;
  }

  // Ensure output directory exists
  auto parent = std::filesystem::path(args.output_path).parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);

  pcl::io::savePCDFileBinary(args.output_path, *merged);
  std::cerr << "Saved: " << args.output_path << " (" << merged->size() << " points)\n";
  return 0;
}

// ============================================================================
// graph
// ============================================================================

static int cmdGraph(const std::string & path)
{
  auto db = openMap(path);
  if (!db) return 1;

  std::cout << "key_a,key_b,owner\n";

  auto stmt = prepare(db.get(), "SELECT key_a, key_b, owner FROM edges");
  while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
    int64_t ka = sqlite3_column_int64(stmt.get(), 0);
    int64_t kb = sqlite3_column_int64(stmt.get(), 1);
    auto owner_col = sqlite3_column_text(stmt.get(), 2);
    std::string owner = owner_col ? reinterpret_cast<const char *>(owner_col) : "";
    std::cout << ka << "," << kb << "," << owner << "\n";
  }

  return 0;
}

// ============================================================================
// main
// ============================================================================

int main(int argc, char ** argv)
{
  if (argc < 2) {
    printUsage();
    return 1;
  }

  std::string cmd(argv[1]);

  if (cmd == "info") {
    if (argc < 3) {
      std::cerr << "Usage: eidos info <map_file>\n";
      return 1;
    }
    return cmdInfo(argv[2]);
  }

  if (cmd == "poses") {
    if (argc < 3) {
      std::cerr << "Usage: eidos poses <map_file>\n";
      return 1;
    }
    return cmdPoses(argv[2]);
  }

  if (cmd == "map") {
    return cmdMap(argc, argv);
  }

  if (cmd == "graph") {
    if (argc < 3) {
      std::cerr << "Usage: eidos graph <map_file>\n";
      return 1;
    }
    return cmdGraph(argv[2]);
  }

  if (cmd == "--help" || cmd == "-h" || cmd == "help") {
    printUsage();
    return 0;
  }

  std::cerr << "Unknown command: " << cmd << "\n";
  printUsage();
  return 1;
}
