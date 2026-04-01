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
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <small_gicp/points/point_cloud.hpp>

#include "eidos/map/registry.hpp"
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
            << "  graph <map_file>                                   Print factor graph edges\n"
            << "  convert --input <in.osm> --output <out.osm>         Convert local to WGS84\n"
            << "          --origin-lat <lat> --origin-lon <lon>\n";
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
// convert
// ============================================================================

struct ConvertArgs
{
  std::string input_path;
  std::string output_path;
  double origin_lat = 0.0;
  double origin_lon = 0.0;
};

static bool parseConvertArgs(int argc, char ** argv, ConvertArgs & args)
{
  for (int i = 2; i < argc; i++) {
    std::string arg(argv[i]);
    if ((arg == "--input" || arg == "-i") && i + 1 < argc) {
      args.input_path = argv[++i];
    } else if ((arg == "--output" || arg == "-o") && i + 1 < argc) {
      args.output_path = argv[++i];
    } else if (arg == "--origin-lat" && i + 1 < argc) {
      args.origin_lat = std::stod(argv[++i]);
    } else if (arg == "--origin-lon" && i + 1 < argc) {
      args.origin_lon = std::stod(argv[++i]);
    }
  }

  if (args.input_path.empty()) {
    std::cerr << "Error: --input is required\n";
    return false;
  }
  if (args.output_path.empty()) {
    std::cerr << "Error: --output is required\n";
    return false;
  }
  if (args.origin_lat == 0.0 && args.origin_lon == 0.0) {
    std::cerr << "Error: --origin-lat and --origin-lon are required\n";
    return false;
  }
  return true;
}

/// UTM forward projection (WGS84 lat/lon -> UTM easting/northing).
static void latlonToUtm(double lat_deg, double lon_deg, double & easting, double & northing, int & zone)
{
  constexpr double a = 6378137.0;
  constexpr double f = 1.0 / 298.257223563;
  constexpr double e2 = 2.0 * f - f * f;
  constexpr double k0 = 0.9996;

  double lat = lat_deg * M_PI / 180.0;
  double lon = lon_deg * M_PI / 180.0;

  zone = static_cast<int>((lon_deg + 180.0) / 6.0) + 1;
  double lon0 = ((zone - 1) * 6.0 - 180.0 + 3.0) * M_PI / 180.0;

  double sinLat = std::sin(lat);
  double cosLat = std::cos(lat);
  double tanLat = std::tan(lat);

  double N = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
  double T = tanLat * tanLat;
  double C = (e2 / (1.0 - e2)) * cosLat * cosLat;
  double A = cosLat * (lon - lon0);

  double M = a * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * lat -
                  (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(2.0 * lat) +
                  (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(4.0 * lat) -
                  (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * lat));

  double ep2 = e2 / (1.0 - e2);
  easting = k0 * N *
              (A + (1.0 - T + C) * A * A * A / 6.0 +
               (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * A * A * A * A * A / 120.0) +
            500000.0;
  northing = k0 * (M + N * tanLat *
                         (A * A / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0 +
                          (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * A * A * A * A * A * A / 720.0));
}

/// UTM inverse projection (UTM easting/northing -> WGS84 lat/lon).
static void utmToLatlon(double easting, double northing, int zone, bool is_north, double & lat_deg, double & lon_deg)
{
  constexpr double a = 6378137.0;
  constexpr double f = 1.0 / 298.257223563;
  constexpr double e2 = 2.0 * f - f * f;
  constexpr double ep2 = e2 / (1.0 - e2);
  constexpr double k0 = 0.9996;

  double x = easting - 500000.0;
  double y = is_north ? northing : northing - 10000000.0;

  double M = y / k0;
  double mu = M / (a * (1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0));

  double e1 = (1.0 - std::sqrt(1.0 - e2)) / (1.0 + std::sqrt(1.0 - e2));
  double phi1 = mu + (3.0 * e1 / 2.0 - 27.0 * e1 * e1 * e1 / 32.0) * std::sin(2.0 * mu) +
                (21.0 * e1 * e1 / 16.0 - 55.0 * e1 * e1 * e1 * e1 / 32.0) * std::sin(4.0 * mu) +
                (151.0 * e1 * e1 * e1 / 96.0) * std::sin(6.0 * mu) +
                (1097.0 * e1 * e1 * e1 * e1 / 512.0) * std::sin(8.0 * mu);

  double sinP = std::sin(phi1);
  double cosP = std::cos(phi1);
  double tanP = std::tan(phi1);

  double N = a / std::sqrt(1.0 - e2 * sinP * sinP);
  double T = tanP * tanP;
  double C = ep2 * cosP * cosP;
  double R = a * (1.0 - e2) / std::pow(1.0 - e2 * sinP * sinP, 1.5);
  double D = x / (N * k0);

  double lat =
    phi1 - (N * tanP / R) *
             (D * D / 2.0 - (5.0 + 3.0 * T + 10.0 * C - 4.0 * C * C - 9.0 * ep2) * D * D * D * D / 24.0 +
              (61.0 + 90.0 * T + 298.0 * C + 45.0 * T * T - 252.0 * ep2 - 3.0 * C * C) * D * D * D * D * D * D / 720.0);

  double lon0 = ((zone - 1) * 6.0 - 180.0 + 3.0) * M_PI / 180.0;
  double lon =
    lon0 + (D - (1.0 + 2.0 * T + C) * D * D * D / 6.0 +
            (5.0 - 2.0 * C + 28.0 * T - 3.0 * C * C + 8.0 * ep2 + 24.0 * T * T) * D * D * D * D * D / 120.0) /
             cosP;

  lat_deg = lat * 180.0 / M_PI;
  lon_deg = lon * 180.0 / M_PI;
}

static int cmdConvert(int argc, char ** argv)
{
  ConvertArgs args;
  if (!parseConvertArgs(argc, argv, args)) {
    std::cerr << "Usage: eidos convert --input <in.osm> --output <out.osm> "
                 "--origin-lat <lat> --origin-lon <lon>\n"
              << "\n"
              << "Converts a lanelet2 OSM file from local coordinates (local_x/local_y)\n"
              << "to WGS84 lat/lon. The origin must match the UtmProjector origin used\n"
              << "by the world model.\n";
    return 1;
  }

  if (!std::filesystem::exists(args.input_path)) {
    std::cerr << "Error: file not found: " << args.input_path << "\n";
    return 1;
  }

  // Compute origin UTM
  double origin_e, origin_n;
  int zone;
  latlonToUtm(args.origin_lat, args.origin_lon, origin_e, origin_n, zone);

  std::cerr << "Origin: (" << std::fixed << std::setprecision(10) << args.origin_lat << ", " << args.origin_lon
            << ")\n";
  std::cerr << "UTM zone " << zone << ": (" << std::setprecision(4) << origin_e << ", " << origin_n << ")\n";

  // Parse XML
  // We do a simple line-by-line approach: for each <node> with local_x/local_y tags,
  // compute and set lat/lon. We use a minimal XML parser via tinyxml-style reading.
  // Actually, let's just use a simple approach: read file, find nodes, update lat/lon.

  // Read entire file
  std::ifstream ifs(args.input_path);
  if (!ifs.is_open()) {
    std::cerr << "Error: cannot open " << args.input_path << "\n";
    return 1;
  }
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  ifs.close();

  // Parse with a simple state machine:
  // Find each <node ...> block, extract local_x/local_y, compute lat/lon, replace lat="" lon=""
  // This avoids adding an XML library dependency.

  std::string output;
  output.reserve(content.size() * 1.2);

  size_t pos = 0;
  int converted = 0;

  while (pos < content.size()) {
    // Find next <node
    size_t node_start = content.find("<node ", pos);
    if (node_start == std::string::npos) {
      output.append(content, pos, content.size() - pos);
      break;
    }

    // Copy everything before this node tag
    output.append(content, pos, node_start - pos);

    // Find the end of this node's block (either /> or </node>)
    size_t node_block_end = content.find("</node>", node_start);
    size_t self_close = content.find("/>", node_start);

    // The node tag itself ends at the first >
    size_t node_tag_end = content.find('>', node_start);
    if (node_tag_end == std::string::npos) {
      output.append(content, node_start, content.size() - node_start);
      break;
    }

    // Determine the full block end
    size_t block_end;
    if (node_block_end != std::string::npos && (self_close == std::string::npos || node_block_end < self_close + 2)) {
      block_end = node_block_end + 7;  // length of "</node>"
    } else if (self_close != std::string::npos && self_close < node_tag_end + 1) {
      // Self-closing <node ... />
      block_end = self_close + 2;
    } else {
      block_end = node_block_end != std::string::npos ? node_block_end + 7 : content.size();
    }

    std::string block = content.substr(node_start, block_end - node_start);

    // Extract local_x and local_y from tags within this block
    auto extractTagValue = [&](const std::string & blk, const std::string & key) -> double {
      std::string pattern = "k=\"" + key + "\"";
      size_t kpos = blk.find(pattern);
      if (kpos == std::string::npos) return std::numeric_limits<double>::quiet_NaN();
      size_t vpos = blk.find("v=\"", kpos);
      if (vpos == std::string::npos) return std::numeric_limits<double>::quiet_NaN();
      vpos += 3;
      size_t vend = blk.find('"', vpos);
      if (vend == std::string::npos) return std::numeric_limits<double>::quiet_NaN();
      return std::stod(blk.substr(vpos, vend - vpos));
    };

    double lx = extractTagValue(block, "local_x");
    double ly = extractTagValue(block, "local_y");

    if (!std::isnan(lx) && !std::isnan(ly)) {
      // Compute lat/lon
      double lat, lon;
      utmToLatlon(origin_e + lx, origin_n + ly, zone, true, lat, lon);

      // Replace lat="..." and lon="..." in the <node> opening tag
      // Find lat= and lon= within the <node ...> tag (not in child tags)
      size_t tag_end = block.find('>');
      std::string tag = block.substr(0, tag_end);
      std::string rest = block.substr(tag_end);

      // Replace lat="..."
      auto replaceAttr = [](std::string & s, const std::string & attr, const std::string & val) {
        std::string prefix = attr + "=\"";
        size_t start = s.find(prefix);
        if (start == std::string::npos) return;
        start += prefix.size();
        size_t end = s.find('"', start);
        if (end == std::string::npos) return;
        s.replace(start, end - start, val);
      };

      std::ostringstream lat_ss, lon_ss;
      lat_ss << std::fixed << std::setprecision(10) << lat;
      lon_ss << std::fixed << std::setprecision(10) << lon;

      replaceAttr(tag, "lat", lat_ss.str());
      replaceAttr(tag, "lon", lon_ss.str());

      output.append(tag);
      output.append(rest);
      converted++;
    } else {
      output.append(block);
    }

    pos = block_end;
  }

  // Write output
  auto out_parent = std::filesystem::path(args.output_path).parent_path();
  if (!out_parent.empty()) std::filesystem::create_directories(out_parent);

  std::ofstream ofs(args.output_path);
  if (!ofs.is_open()) {
    std::cerr << "Error: cannot write " << args.output_path << "\n";
    return 1;
  }
  ofs << output;
  ofs.close();

  std::cerr << "Converted " << converted << " nodes -> " << args.output_path << "\n";
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

  if (cmd == "convert") {
    return cmdConvert(argc, argv);
  }

  if (cmd == "--help" || cmd == "-h" || cmd == "help") {
    printUsage();
    return 0;
  }

  std::cerr << "Unknown command: " << cmd << "\n";
  printUsage();
  return 1;
}
