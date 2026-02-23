#include <catch2/catch_all.hpp>

#include "eidos/scan_matcher.hpp"

using namespace eidos;

namespace {

pcl::PointCloud<PointType>::Ptr makeSurfaceCloud(
    int n, float x_offset = 0.0f, float y_offset = 0.0f) {
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      PointType pt;
      pt.x = static_cast<float>(i) * 0.5f + x_offset;
      pt.y = static_cast<float>(j) * 0.5f + y_offset;
      pt.z = 0.0f;
      pt.intensity = 1.0f;
      cloud->push_back(pt);
    }
  }
  return cloud;
}

pcl::PointCloud<PointType>::Ptr makeEdgeCloud(
    int n, float x_offset = 0.0f) {
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  for (int i = 0; i < n; i++) {
    PointType pt;
    pt.x = static_cast<float>(i) * 0.1f + x_offset;
    pt.y = 0.0f;
    pt.z = static_cast<float>(i) * 0.1f;
    pt.intensity = 1.0f;
    cloud->push_back(pt);
  }
  return cloud;
}

}  // namespace

TEST_CASE("ScanMatcher rejects insufficient features", "[scan_matcher]") {
  ScanMatcher::Config config;
  config.edge_feature_min_valid = 5;
  config.surf_feature_min_valid = 10;
  config.max_iterations = 10;
  config.num_cores = 1;
  ScanMatcher matcher(config);

  auto corners = makeEdgeCloud(2);      // less than min valid (5)
  auto surfaces = makeSurfaceCloud(2);  // 4 points, less than min valid (10)

  auto corner_map = makeEdgeCloud(100);
  auto surface_map = makeSurfaceCloud(20);

  auto corner_kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  corner_kdtree->setInputCloud(corner_map);
  auto surface_kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  surface_kdtree->setInputCloud(surface_map);

  float initial[6] = {0, 0, 0, 0, 0, 0};
  auto result = matcher.match(corners, surfaces, corner_map, surface_map,
                               corner_kdtree, surface_kdtree, initial);

  REQUIRE_FALSE(result.converged);
}

TEST_CASE("ScanMatcher identity match", "[scan_matcher]") {
  ScanMatcher::Config config;
  config.edge_feature_min_valid = 5;
  config.surf_feature_min_valid = 10;
  config.max_iterations = 10;
  config.num_cores = 1;
  ScanMatcher matcher(config);

  auto corners = makeEdgeCloud(20);
  auto surfaces = makeSurfaceCloud(15);  // 15x15 = 225 points

  auto corner_kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  corner_kdtree->setInputCloud(corners);
  auto surface_kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  surface_kdtree->setInputCloud(surfaces);

  float initial[6] = {0, 0, 0, 0, 0, 0};
  auto result = matcher.match(corners, surfaces, corners, surfaces,
                               corner_kdtree, surface_kdtree, initial);

  // With identity initial guess and same cloud, transform should stay near zero
  REQUIRE(result.transform[0] == Catch::Approx(0.0f).margin(0.5f));
  REQUIRE(result.transform[1] == Catch::Approx(0.0f).margin(0.5f));
  REQUIRE(result.transform[2] == Catch::Approx(0.0f).margin(0.5f));
  REQUIRE(result.transform[3] == Catch::Approx(0.0f).margin(0.5f));
  REQUIRE(result.transform[4] == Catch::Approx(0.0f).margin(0.5f));
  REQUIRE(result.transform[5] == Catch::Approx(0.0f).margin(0.5f));
}

TEST_CASE("ScanMatcher result has no NaN", "[scan_matcher]") {
  ScanMatcher::Config config;
  config.edge_feature_min_valid = 5;
  config.surf_feature_min_valid = 10;
  config.max_iterations = 10;
  config.num_cores = 1;
  ScanMatcher matcher(config);

  auto corners = makeEdgeCloud(20);
  auto surfaces = makeSurfaceCloud(15);

  auto corner_kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  corner_kdtree->setInputCloud(corners);
  auto surface_kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  surface_kdtree->setInputCloud(surfaces);

  float initial[6] = {0.1f, 0.2f, 0.05f, 1.0f, 2.0f, 0.5f};
  auto result = matcher.match(corners, surfaces, corners, surfaces,
                               corner_kdtree, surface_kdtree, initial);

  for (int i = 0; i < 6; i++) {
    REQUIRE_FALSE(std::isnan(result.transform[i]));
  }
}
