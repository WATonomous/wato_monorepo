#include <catch2/catch_all.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <pcl/io/pcd_io.h>

#include "eidos/map_manager.hpp"

using namespace eidos;
using gtsam::Symbol;

namespace {

PoseType makePose(float x, float y, float z, double time, int index) {
  PoseType p;
  p.x = x; p.y = y; p.z = z;
  p.roll = 0; p.pitch = 0; p.yaw = 0;
  p.time = time;
  p.intensity = static_cast<float>(index);
  return p;
}

pcl::PointCloud<PointType>::Ptr makeCloud(int n, float offset = 0.0f) {
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  for (int i = 0; i < n; i++) {
    PointType pt;
    pt.x = static_cast<float>(i) * 0.1f + offset;
    pt.y = static_cast<float>(i) * 0.2f + offset;
    pt.z = 0.0f;
    pt.intensity = 1.0f;
    cloud->push_back(pt);
  }
  return cloud;
}

}  // namespace

TEST_CASE("MapManager starts empty", "[map_manager]") {
  MapManager mm;
  REQUIRE(mm.numKeyframes() == 0);
  REQUIRE(mm.getKeyPoses3D()->empty());
  REQUIRE(mm.getKeyPoses6D()->empty());
  REQUIRE_FALSE(mm.hasPriorMap());
}

TEST_CASE("MapManager addKeyframe", "[map_manager]") {
  MapManager mm;
  PoseType pose = makePose(1.0, 2.0, 3.0, 0.0, 0);

  gtsam::Key key0 = Symbol(255, 0);
  mm.addKeyframe(key0, pose);

  REQUIRE(mm.numKeyframes() == 1);
  REQUIRE(mm.getKeyPoses3D()->size() == 1);
  REQUIRE(mm.getKeyPoses6D()->size() == 1);

  auto poses3d = mm.getKeyPoses3D();
  REQUIRE(poses3d->points[0].x == Catch::Approx(1.0f));
  REQUIRE(poses3d->points[0].y == Catch::Approx(2.0f));
  REQUIRE(poses3d->points[0].z == Catch::Approx(3.0f));
}

TEST_CASE("MapManager add multiple keyframes", "[map_manager]") {
  MapManager mm;
  for (int i = 0; i < 5; i++) {
    PoseType pose = makePose(static_cast<float>(i), 0, 0,
                             static_cast<double>(i), i);
    mm.addKeyframe(Symbol(255, i), pose);
  }
  REQUIRE(mm.numKeyframes() == 5);
}

// ---- Per-keyframe data storage tests ----

TEST_CASE("MapManager addKeyframeData and getKeyframeData", "[map_manager]") {
  MapManager mm;
  gtsam::Key key0 = Symbol(255, 0);
  PoseType pose = makePose(0, 0, 0, 0.0, 0);
  mm.addKeyframe(key0, pose);

  auto cloud = makeCloud(10);
  mm.addKeyframeData(key0, "test_plugin/corners", cloud);

  auto result = mm.getKeyframeData(key0, "test_plugin/corners");
  REQUIRE(result.has_value());

  auto retrieved = std::any_cast<pcl::PointCloud<PointType>::Ptr>(result.value());
  REQUIRE(retrieved->size() == 10);
}

TEST_CASE("MapManager getKeyframeData returns nullopt for missing data", "[map_manager]") {
  MapManager mm;
  gtsam::Key key0 = Symbol(255, 0);
  PoseType pose = makePose(0, 0, 0, 0.0, 0);
  mm.addKeyframe(key0, pose);

  auto result = mm.getKeyframeData(key0, "test_plugin/corners");
  REQUIRE_FALSE(result.has_value());

  auto result2 = mm.getKeyframeData(Symbol(255, 99), "test_plugin/corners");
  REQUIRE_FALSE(result2.has_value());
}

TEST_CASE("MapManager getKeyframeDataForPlugin", "[map_manager]") {
  MapManager mm;
  gtsam::Key key0 = Symbol(255, 0);
  PoseType pose = makePose(0, 0, 0, 0.0, 0);
  mm.addKeyframe(key0, pose);

  auto corners = makeCloud(10);
  auto surfaces = makeCloud(20);
  auto other = makeCloud(5);
  mm.addKeyframeData(key0, "lidar_kep_factor/corners", corners);
  mm.addKeyframeData(key0, "lidar_kep_factor/surfaces", surfaces);
  mm.addKeyframeData(key0, "other_plugin/data", other);

  auto plugin_data = mm.getKeyframeDataForPlugin(key0, "lidar_kep_factor");
  REQUIRE(plugin_data.size() == 2);
  REQUIRE(plugin_data.count("lidar_kep_factor/corners") == 1);
  REQUIRE(plugin_data.count("lidar_kep_factor/surfaces") == 1);

  auto c = std::any_cast<pcl::PointCloud<PointType>::Ptr>(
      plugin_data.at("lidar_kep_factor/corners"));
  REQUIRE(c->size() == 10);
}

// ---- Persistence tests ----

TEST_CASE("MapManager save and load poses + keys", "[map_manager]") {
  std::string test_dir = "/tmp/eidos_test_map_" +
      std::to_string(std::chrono::system_clock::now().time_since_epoch().count());

  {
    MapManager mm;
    for (int i = 0; i < 3; i++) {
      gtsam::Key key = Symbol('x', i);
      PoseType pose = makePose(static_cast<float>(i), 0, 0,
                               static_cast<double>(i), i);
      mm.addKeyframe(key, pose, "liso_factor");
    }

    std::vector<std::string> plugins = {"liso_factor", "gps_factor"};
    REQUIRE(mm.saveMap(test_dir, plugins));

    REQUIRE(std::filesystem::exists(test_dir + "/poses.pcd"));
    REQUIRE(std::filesystem::exists(test_dir + "/keys.bin"));
    REQUIRE(std::filesystem::exists(test_dir + "/metadata.yaml"));
  }

  // Load into a fresh MapManager
  {
    MapManager mm2;
    std::vector<std::string> saved_plugins;
    REQUIRE(mm2.loadMap(test_dir, saved_plugins));
    REQUIRE(mm2.numKeyframes() == 3);
    REQUIRE(mm2.hasPriorMap());

    // Verify original keys preserved
    auto key_list = mm2.getKeyList();
    REQUIRE(key_list[0] == Symbol('x', 0));
    REQUIRE(key_list[1] == Symbol('x', 1));
    REQUIRE(key_list[2] == Symbol('x', 2));

    // Verify owner preserved
    REQUIRE(mm2.getOwnerPlugin(Symbol('x', 0)) == "liso_factor");

    // Verify poses
    auto poses3d = mm2.getKeyPoses3D();
    REQUIRE(poses3d->points[0].x == Catch::Approx(0.0f));
    REQUIRE(poses3d->points[1].x == Catch::Approx(1.0f));
    REQUIRE(poses3d->points[2].x == Catch::Approx(2.0f));

    // Verify plugin names from metadata
    REQUIRE(saved_plugins.size() == 2);
    REQUIRE(saved_plugins[0] == "liso_factor");
    REQUIRE(saved_plugins[1] == "gps_factor");
  }

  std::filesystem::remove_all(test_dir);
}

TEST_CASE("MapManager load nonexistent directory", "[map_manager]") {
  MapManager mm;
  std::vector<std::string> plugins;
  REQUIRE_FALSE(mm.loadMap("/tmp/nonexistent_eidos_test_dir_12345", plugins));
}

TEST_CASE("MapManager updatePoses", "[map_manager]") {
  MapManager mm;
  for (int i = 0; i < 3; i++) {
    PoseType pose = makePose(static_cast<float>(i), 0, 0,
                             static_cast<double>(i), i);
    mm.addKeyframe(Symbol(255, i), pose);
  }

  gtsam::Values optimized;
  for (int i = 0; i < 3; i++) {
    optimized.insert(Symbol(255, i), gtsam::Pose3(
        gtsam::Rot3(), gtsam::Point3(i + 10.0, 0, 0)));
  }

  mm.updatePoses(optimized);

  auto poses3d = mm.getKeyPoses3D();
  REQUIRE(poses3d->points[0].x == Catch::Approx(10.0f).margin(0.01f));
  REQUIRE(poses3d->points[1].x == Catch::Approx(11.0f).margin(0.01f));
  REQUIRE(poses3d->points[2].x == Catch::Approx(12.0f).margin(0.01f));
}
