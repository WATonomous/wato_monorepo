#include <catch2/catch_all.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <pcl/io/pcd_io.h>

#include "eidos/map_manager.hpp"

using namespace eidos;

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

/// Helper to register a point cloud type with PCD serialize/deserialize.
void registerCloudType(MapManager& mm, const std::string& key) {
  MapManager::TypeHandler handler;
  handler.serialize = [](const std::any& data, const std::string& path) {
    auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
    if (cloud && !cloud->empty()) {
      pcl::io::savePCDFileBinary(path, *cloud);
    }
  };
  handler.deserialize = [](const std::string& path) -> std::any {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
    pcl::io::loadPCDFile(path, *cloud);
    return cloud;
  };
  mm.registerType(key, std::move(handler));
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

  mm.addKeyframe(0, pose);

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
    mm.addKeyframe(i, pose);
  }
  REQUIRE(mm.numKeyframes() == 5);
}

// ---- Type registry tests ----

TEST_CASE("MapManager registerType and hasType", "[map_manager]") {
  MapManager mm;
  REQUIRE_FALSE(mm.hasType("test_plugin/data"));

  registerCloudType(mm, "test_plugin/data");
  REQUIRE(mm.hasType("test_plugin/data"));
  REQUIRE_FALSE(mm.hasType("other_plugin/data"));
}

TEST_CASE("MapManager getKeysForPlugin", "[map_manager]") {
  MapManager mm;
  registerCloudType(mm, "lidar_kep_factor/corners");
  registerCloudType(mm, "lidar_kep_factor/surfaces");
  registerCloudType(mm, "other_plugin/data");

  auto keys = mm.getKeysForPlugin("lidar_kep_factor");
  REQUIRE(keys.size() == 2);
  REQUIRE(keys[0] == "lidar_kep_factor/corners");
  REQUIRE(keys[1] == "lidar_kep_factor/surfaces");

  auto other_keys = mm.getKeysForPlugin("other_plugin");
  REQUIRE(other_keys.size() == 1);

  auto empty_keys = mm.getKeysForPlugin("nonexistent");
  REQUIRE(empty_keys.empty());
}

// ---- Per-keyframe data storage tests ----

TEST_CASE("MapManager addKeyframeData and getKeyframeData", "[map_manager]") {
  MapManager mm;
  registerCloudType(mm, "test_plugin/corners");

  PoseType pose = makePose(0, 0, 0, 0.0, 0);
  mm.addKeyframe(0, pose);

  auto cloud = makeCloud(10);
  mm.addKeyframeData(0, "test_plugin/corners", cloud);

  auto result = mm.getKeyframeData(0, "test_plugin/corners");
  REQUIRE(result.has_value());

  auto retrieved = std::any_cast<pcl::PointCloud<PointType>::Ptr>(result.value());
  REQUIRE(retrieved->size() == 10);
}

TEST_CASE("MapManager getKeyframeData returns nullopt for missing data", "[map_manager]") {
  MapManager mm;
  PoseType pose = makePose(0, 0, 0, 0.0, 0);
  mm.addKeyframe(0, pose);

  // No data stored yet
  auto result = mm.getKeyframeData(0, "test_plugin/corners");
  REQUIRE_FALSE(result.has_value());

  // Out of range index
  auto result2 = mm.getKeyframeData(99, "test_plugin/corners");
  REQUIRE_FALSE(result2.has_value());

  // Negative index
  auto result3 = mm.getKeyframeData(-1, "test_plugin/corners");
  REQUIRE_FALSE(result3.has_value());
}

TEST_CASE("MapManager getKeyframeDataForPlugin", "[map_manager]") {
  MapManager mm;
  registerCloudType(mm, "lidar_kep_factor/corners");
  registerCloudType(mm, "lidar_kep_factor/surfaces");
  registerCloudType(mm, "other_plugin/data");

  PoseType pose = makePose(0, 0, 0, 0.0, 0);
  mm.addKeyframe(0, pose);

  auto corners = makeCloud(10);
  auto surfaces = makeCloud(20);
  auto other = makeCloud(5);
  mm.addKeyframeData(0, "lidar_kep_factor/corners", corners);
  mm.addKeyframeData(0, "lidar_kep_factor/surfaces", surfaces);
  mm.addKeyframeData(0, "other_plugin/data", other);

  auto plugin_data = mm.getKeyframeDataForPlugin(0, "lidar_kep_factor");
  REQUIRE(plugin_data.size() == 2);
  REQUIRE(plugin_data.count("lidar_kep_factor/corners") == 1);
  REQUIRE(plugin_data.count("lidar_kep_factor/surfaces") == 1);

  auto c = std::any_cast<pcl::PointCloud<PointType>::Ptr>(
      plugin_data.at("lidar_kep_factor/corners"));
  REQUIRE(c->size() == 10);

  auto s = std::any_cast<pcl::PointCloud<PointType>::Ptr>(
      plugin_data.at("lidar_kep_factor/surfaces"));
  REQUIRE(s->size() == 20);
}

TEST_CASE("MapManager getKeyframeDataForPlugin empty for out of range", "[map_manager]") {
  MapManager mm;
  auto result = mm.getKeyframeDataForPlugin(0, "lidar_kep_factor");
  REQUIRE(result.empty());
}

// ---- Persistence tests ----

TEST_CASE("MapManager save and load map", "[map_manager]") {
  std::string test_dir = "/tmp/eidos_test_map_" +
      std::to_string(std::chrono::system_clock::now().time_since_epoch().count());

  {
    MapManager mm;
    registerCloudType(mm, "test_plugin/corners");
    registerCloudType(mm, "test_plugin/surfaces");

    for (int i = 0; i < 3; i++) {
      PoseType pose = makePose(static_cast<float>(i), 0, 0,
                               static_cast<double>(i), i);
      mm.addKeyframe(i, pose);

      auto corners = makeCloud(10, static_cast<float>(i));
      auto surfaces = makeCloud(20, static_cast<float>(i));
      mm.addKeyframeData(i, "test_plugin/corners", corners);
      mm.addKeyframeData(i, "test_plugin/surfaces", surfaces);
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
    for (int i = 0; i < 3; i++) {
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(
          i, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(i, 0, 0)), noise));
      values.insert(i, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(i, 0, 0)));
    }

    REQUIRE(mm.saveMap(test_dir, 0.0f, graph, values));

    REQUIRE(std::filesystem::exists(test_dir + "/trajectory.pcd"));
    REQUIRE(std::filesystem::exists(test_dir + "/transformations.pcd"));
    REQUIRE(std::filesystem::exists(test_dir + "/metadata.yaml"));
    REQUIRE(std::filesystem::exists(
        test_dir + "/keyframes/000000/test_plugin/corners.bin"));
    REQUIRE(std::filesystem::exists(
        test_dir + "/keyframes/000000/test_plugin/surfaces.bin"));
  }

  // Load into a fresh MapManager (must register the same types first)
  {
    MapManager mm2;
    registerCloudType(mm2, "test_plugin/corners");
    registerCloudType(mm2, "test_plugin/surfaces");

    REQUIRE(mm2.loadMap(test_dir));
    REQUIRE(mm2.numKeyframes() == 3);
    REQUIRE(mm2.hasPriorMap());

    auto poses3d = mm2.getKeyPoses3D();
    REQUIRE(poses3d->points[0].x == Catch::Approx(0.0f));
    REQUIRE(poses3d->points[1].x == Catch::Approx(1.0f));
    REQUIRE(poses3d->points[2].x == Catch::Approx(2.0f));

    // Verify per-keyframe data was restored
    auto result = mm2.getKeyframeData(0, "test_plugin/corners");
    REQUIRE(result.has_value());
    auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(result.value());
    REQUIRE(cloud->size() == 10);

    auto result2 = mm2.getKeyframeData(1, "test_plugin/surfaces");
    REQUIRE(result2.has_value());
    auto cloud2 = std::any_cast<pcl::PointCloud<PointType>::Ptr>(result2.value());
    REQUIRE(cloud2->size() == 20);
  }

  std::filesystem::remove_all(test_dir);
}

TEST_CASE("MapManager load nonexistent directory", "[map_manager]") {
  MapManager mm;
  REQUIRE_FALSE(mm.loadMap("/tmp/nonexistent_eidos_test_dir_12345"));
}

TEST_CASE("MapManager load without registered types skips data", "[map_manager]") {
  std::string test_dir = "/tmp/eidos_test_map_notype_" +
      std::to_string(std::chrono::system_clock::now().time_since_epoch().count());

  // Save with registered types
  {
    MapManager mm;
    registerCloudType(mm, "test_plugin/data");
    PoseType pose = makePose(1, 2, 3, 0.0, 0);
    mm.addKeyframe(0, pose);
    mm.addKeyframeData(0, "test_plugin/data", makeCloud(10));

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        0, gtsam::Pose3(), noise));
    values.insert(0, gtsam::Pose3());
    mm.saveMap(test_dir, 0.0f, graph, values);
  }

  // Load without registering the type — poses should still load
  {
    MapManager mm2;
    REQUIRE(mm2.loadMap(test_dir));
    REQUIRE(mm2.numKeyframes() == 1);
    REQUIRE(mm2.hasPriorMap());

    // Data should not be available since we didn't register the deserializer
    auto result = mm2.getKeyframeData(0, "test_plugin/data");
    REQUIRE_FALSE(result.has_value());
  }

  std::filesystem::remove_all(test_dir);
}

TEST_CASE("MapManager updatePoses", "[map_manager]") {
  MapManager mm;
  for (int i = 0; i < 3; i++) {
    PoseType pose = makePose(static_cast<float>(i), 0, 0,
                             static_cast<double>(i), i);
    mm.addKeyframe(i, pose);
  }

  gtsam::Values optimized;
  for (int i = 0; i < 3; i++) {
    optimized.insert(i, gtsam::Pose3(
        gtsam::Rot3(), gtsam::Point3(i + 10.0, 0, 0)));
  }

  mm.updatePoses(optimized);

  auto poses3d = mm.getKeyPoses3D();
  REQUIRE(poses3d->points[0].x == Catch::Approx(10.0f).margin(0.01f));
  REQUIRE(poses3d->points[1].x == Catch::Approx(11.0f).margin(0.01f));
  REQUIRE(poses3d->points[2].x == Catch::Approx(12.0f).margin(0.01f));
}
