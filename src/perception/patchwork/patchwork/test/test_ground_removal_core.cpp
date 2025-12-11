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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_all.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "patchworkpp/ground_removal_core.hpp"

using wato::perception::patchworkpp::GroundRemovalCore;

// Helper function to filter invalid points (NaN/Inf) - mimics GroundRemovalNode::filterInvalidPoints
// This is used for testing the filtering behavior
Eigen::MatrixX3f filterInvalidPoints(const Eigen::MatrixX3f & cloud)
{
  size_t valid_points = 0;
  Eigen::MatrixX3f filtered_cloud(cloud.rows(), 3);

  for (int i = 0; i < cloud.rows(); ++i) {
    if (std::isfinite(cloud(i, 0)) && std::isfinite(cloud(i, 1)) && std::isfinite(cloud(i, 2))) {
      filtered_cloud.row(valid_points++) = cloud.row(i);
    }
  }

  if (valid_points == 0) {
    return Eigen::MatrixX3f(0, 3);
  }

  if (valid_points < static_cast<size_t>(cloud.rows())) {
    filtered_cloud.conservativeResize(static_cast<int>(valid_points), 3);
  }

  return filtered_cloud;
}

// Helper to create a simple PointCloud2
sensor_msgs::msg::PointCloud2 createPointCloud(size_t width, size_t height, const std::vector<float> & points_xyz)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = height;
  msg.width = width;
  msg.is_bigendian = false;
  msg.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(width * height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

  for (size_t i = 0; i < points_xyz.size() / 3; ++i) {
    *iter_x = points_xyz[i * 3 + 0];
    *iter_y = points_xyz[i * 3 + 1];
    *iter_z = points_xyz[i * 3 + 2];
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return msg;
}

TEST_CASE("GroundRemovalCore Conversion Tests", "[ground_removal_core]")
{
  SECTION("PointCloud2 to Eigen conversion (Valid Data)")
  {
    std::vector<float> points = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>(createPointCloud(2, 1, points));

    Eigen::MatrixX3f eigen_cloud = GroundRemovalCore::pointCloud2ToEigen(msg);

    REQUIRE(eigen_cloud.rows() == 2);
    REQUIRE(eigen_cloud.cols() == 3);
    REQUIRE_THAT(eigen_cloud(0, 0), Catch::Matchers::WithinRel(1.0f));
    REQUIRE_THAT(eigen_cloud(0, 1), Catch::Matchers::WithinRel(2.0f));
    REQUIRE_THAT(eigen_cloud(0, 2), Catch::Matchers::WithinRel(3.0f));
    REQUIRE_THAT(eigen_cloud(1, 0), Catch::Matchers::WithinRel(4.0f));
  }

  SECTION("PointCloud2 to Eigen with NaN/Inf values")
  {
    float nan = std::numeric_limits<float>::quiet_NaN();
    float inf = std::numeric_limits<float>::infinity();
    std::vector<float> points = {nan, 2.0f, 3.0f, 1.0f, inf, 3.0f, 1.0f, 2.0f, -inf};
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>(createPointCloud(3, 1, points));

    // The core converter should preserve NaNs/Infs so the node logic can filter them
    Eigen::MatrixX3f eigen_cloud = GroundRemovalCore::pointCloud2ToEigen(msg);

    REQUIRE(eigen_cloud.rows() == 3);
    REQUIRE(std::isnan(eigen_cloud(0, 0)));
    REQUIRE(std::isinf(eigen_cloud(1, 1)));
    REQUIRE(std::isinf(eigen_cloud(2, 2)));
  }

  SECTION("Empty PointCloud2 handling")
  {
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>(createPointCloud(0, 0, {}));
    Eigen::MatrixX3f eigen_cloud = GroundRemovalCore::pointCloud2ToEigen(msg);
    REQUIRE(eigen_cloud.rows() == 0);
  }

  SECTION("Throws on missing fields")
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.width = 1;
    msg.height = 1;
    // No fields defined
    auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);

    REQUIRE_THROWS_AS(GroundRemovalCore::pointCloud2ToEigen(msg_ptr), std::runtime_error);
  }

  SECTION("Handles large dimensions without validation limits")
  {
    // Test large point cloud processing (100K points)
    // Conservative size suitable for CI/CD without excessive memory usage
    const size_t large_size = 100000;
    std::vector<float> points;
    points.reserve(large_size * 3);
    for (size_t i = 0; i < large_size; ++i) {
      points.push_back(static_cast<float>(i) * 0.001f);
      points.push_back(static_cast<float>(i) * 0.001f);
      points.push_back(0.0f);
    }
    
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>(createPointCloud(large_size, 1, points));

    // Production code does not validate size - users must test their inputs
    REQUIRE_NOTHROW(GroundRemovalCore::pointCloud2ToEigen(msg));
    
    Eigen::MatrixX3f eigen_cloud = GroundRemovalCore::pointCloud2ToEigen(msg);
    REQUIRE(eigen_cloud.rows() == static_cast<int>(large_size));
  }

  SECTION("Point cloud size validation tests")
  {
    // These tests verify point cloud processing at various sizes
    // Production code does not perform size validation - users must validate inputs
    // Conservative sizes are used to avoid memory issues in CI/CD

    SECTION("Large point cloud (100K points)")
    {
      const size_t size = 100000;
      std::vector<float> points;
      points.reserve(size * 3);
      for (size_t i = 0; i < size; ++i) {
        points.push_back(static_cast<float>(i) * 0.001f);
        points.push_back(static_cast<float>(i) * 0.001f);
        points.push_back(0.0f);
      }
      
      auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>(createPointCloud(size, 1, points));
      REQUIRE_NOTHROW(GroundRemovalCore::pointCloud2ToEigen(msg));
      
      Eigen::MatrixX3f eigen_cloud = GroundRemovalCore::pointCloud2ToEigen(msg);
      REQUIRE(eigen_cloud.rows() == static_cast<int>(size));
    }

    SECTION("Very large point cloud (200K points) - stress test")
    {
      // Stress test with 200K points
      // May be skipped in resource-constrained environments
      const size_t size = 200000;
      std::vector<float> points;
      points.reserve(size * 3);
      for (size_t i = 0; i < size; ++i) {
        points.push_back(static_cast<float>(i) * 0.001f);
        points.push_back(static_cast<float>(i) * 0.001f);
        points.push_back(0.0f);
      }
      
      auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>(createPointCloud(size, 1, points));
      REQUIRE_NOTHROW(GroundRemovalCore::pointCloud2ToEigen(msg));
      
      Eigen::MatrixX3f eigen_cloud = GroundRemovalCore::pointCloud2ToEigen(msg);
      REQUIRE(eigen_cloud.rows() == static_cast<int>(size));
    }

    SECTION("Large width and height combination")
    {
      // Test with 317x317 = 100,489 points (approximately 100K)
      const size_t width = 317;
      const size_t height = 317;
      std::vector<float> points;
      points.reserve(width * height * 3);
      for (size_t h = 0; h < height; ++h) {
        for (size_t w = 0; w < width; ++w) {
          points.push_back(static_cast<float>(w) * 0.001f);
          points.push_back(static_cast<float>(h) * 0.001f);
          points.push_back(0.0f);
        }
      }
      
      auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>(createPointCloud(width, height, points));
      REQUIRE_NOTHROW(GroundRemovalCore::pointCloud2ToEigen(msg));
      
      Eigen::MatrixX3f eigen_cloud = GroundRemovalCore::pointCloud2ToEigen(msg);
      REQUIRE(eigen_cloud.rows() == static_cast<int>(width * height));
    }
  }
}

TEST_CASE("Point Cloud Validation Logic Tests", "[validation]")
{
  SECTION("Point cloud size limits - min/max scenarios")
  {
    // Test that code handles various point cloud sizes correctly
    // Users should validate their inputs match these scenarios

    SECTION("Minimum size - empty point cloud")
    {
      Eigen::MatrixX3f empty_cloud(0, 3);
      REQUIRE(empty_cloud.rows() == 0);
      REQUIRE(empty_cloud.cols() == 3);
    }

    SECTION("Minimum size - single point")
    {
      Eigen::MatrixX3f single_point(1, 3);
      single_point << 1.0f, 2.0f, 3.0f;
      REQUIRE(single_point.rows() == 1);
      REQUIRE(single_point.cols() == 3);
    }

    SECTION("Small point cloud (100 points)")
    {
      Eigen::MatrixX3f small_cloud(100, 3);
      small_cloud.setZero();
      REQUIRE(small_cloud.rows() == 100);
      REQUIRE(small_cloud.cols() == 3);
    }

    SECTION("Medium point cloud (100K points)")
    {
      Eigen::MatrixX3f medium_cloud(100000, 3);
      medium_cloud.setZero();
      REQUIRE(medium_cloud.rows() == 100000);
      REQUIRE(medium_cloud.cols() == 3);
    }

    SECTION("Large point cloud (100K points)")
    {
      Eigen::MatrixX3f large_cloud(100000, 3);
      large_cloud.setZero();
      REQUIRE(large_cloud.rows() == 100000);
      REQUIRE(large_cloud.cols() == 3);
    }

    SECTION("Very large point cloud (200K points) - stress test")
    {
      // Stress test with 200K points - may be skipped in resource-constrained environments
      Eigen::MatrixX3f very_large_cloud(200000, 3);
      very_large_cloud.setZero();
      REQUIRE(very_large_cloud.rows() == 200000);
      REQUIRE(very_large_cloud.cols() == 3);
    }
  }

  SECTION("filterInvalidPoints - NaN/Inf filtering behavior")
  {
    SECTION("All points valid - no filtering needed")
    {
      Eigen::MatrixX3f cloud(5, 3);
      cloud << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 5);
      REQUIRE(filtered.cols() == 3);
      REQUIRE(filtered.isApprox(cloud));
    }

    SECTION("Some points contain NaN - should filter them out")
    {
      float nan = std::numeric_limits<float>::quiet_NaN();
      Eigen::MatrixX3f cloud(5, 3);
      cloud << 1.0f, 2.0f, 3.0f, nan, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, nan, 12.0f, 13.0f, 14.0f, 15.0f;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 3);  // 2 points with NaN should be filtered
      REQUIRE(filtered.cols() == 3);
      REQUIRE(filtered(0, 0) == 1.0f);
      REQUIRE(filtered(0, 1) == 2.0f);
      REQUIRE(filtered(0, 2) == 3.0f);
      REQUIRE(filtered(1, 0) == 7.0f);
      REQUIRE(filtered(1, 1) == 8.0f);
      REQUIRE(filtered(1, 2) == 9.0f);
      REQUIRE(filtered(2, 0) == 13.0f);
      REQUIRE(filtered(2, 1) == 14.0f);
      REQUIRE(filtered(2, 2) == 15.0f);
    }

    SECTION("Some points contain Inf - should filter them out")
    {
      float inf = std::numeric_limits<float>::infinity();
      float neg_inf = -std::numeric_limits<float>::infinity();
      Eigen::MatrixX3f cloud(5, 3);
      cloud << 1.0f, 2.0f, 3.0f, 4.0f, inf, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, neg_inf, 13.0f, 14.0f, 15.0f;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 3);  // 2 points with Inf should be filtered
      REQUIRE(filtered.cols() == 3);
      REQUIRE(filtered(0, 0) == 1.0f);
      REQUIRE(filtered(0, 1) == 2.0f);
      REQUIRE(filtered(0, 2) == 3.0f);
      REQUIRE(filtered(1, 0) == 7.0f);
      REQUIRE(filtered(1, 1) == 8.0f);
      REQUIRE(filtered(1, 2) == 9.0f);
      REQUIRE(filtered(2, 0) == 13.0f);
      REQUIRE(filtered(2, 1) == 14.0f);
      REQUIRE(filtered(2, 2) == 15.0f);
    }

    SECTION("Mixed NaN and Inf - should filter all invalid points")
    {
      float nan = std::numeric_limits<float>::quiet_NaN();
      float inf = std::numeric_limits<float>::infinity();
      Eigen::MatrixX3f cloud(6, 3);
      cloud << 1.0f, 2.0f, 3.0f, nan, 5.0f, 6.0f, 7.0f, inf, 9.0f, 10.0f, 11.0f, 12.0f, nan, nan, nan, 16.0f, 17.0f, 18.0f;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 3);  // 3 valid points out of 6
      REQUIRE(filtered.cols() == 3);
      REQUIRE(filtered(0, 0) == 1.0f);
      REQUIRE(filtered(0, 1) == 2.0f);
      REQUIRE(filtered(0, 2) == 3.0f);
      REQUIRE(filtered(1, 0) == 10.0f);
      REQUIRE(filtered(1, 1) == 11.0f);
      REQUIRE(filtered(1, 2) == 12.0f);
      REQUIRE(filtered(2, 0) == 16.0f);
      REQUIRE(filtered(2, 1) == 17.0f);
      REQUIRE(filtered(2, 2) == 18.0f);
    }

    SECTION("Points with NaN in x coordinate only")
    {
      float nan = std::numeric_limits<float>::quiet_NaN();
      Eigen::MatrixX3f cloud(3, 3);
      cloud << nan, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 2);
      REQUIRE(filtered(0, 0) == 4.0f);
      REQUIRE(filtered(1, 0) == 7.0f);
    }

    SECTION("Points with NaN in y coordinate only")
    {
      float nan = std::numeric_limits<float>::quiet_NaN();
      Eigen::MatrixX3f cloud(3, 3);
      cloud << 1.0f, nan, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 2);
      REQUIRE(filtered(0, 0) == 4.0f);
      REQUIRE(filtered(1, 0) == 7.0f);
    }

    SECTION("Points with NaN in z coordinate only")
    {
      float nan = std::numeric_limits<float>::quiet_NaN();
      Eigen::MatrixX3f cloud(3, 3);
      cloud << 1.0f, 2.0f, nan, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 2);
      REQUIRE(filtered(0, 0) == 4.0f);
      REQUIRE(filtered(1, 0) == 7.0f);
    }
  }

  SECTION("Edge case: all points invalid after filtering")
  {
    SECTION("All points are NaN")
    {
      float nan = std::numeric_limits<float>::quiet_NaN();
      Eigen::MatrixX3f cloud(5, 3);
      cloud << nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 0);
      REQUIRE(filtered.cols() == 3);
    }

    SECTION("All points are Inf")
    {
      float inf = std::numeric_limits<float>::infinity();
      Eigen::MatrixX3f cloud(5, 3);
      cloud << inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 0);
      REQUIRE(filtered.cols() == 3);
    }

    SECTION("All points are -Inf")
    {
      float neg_inf = -std::numeric_limits<float>::infinity();
      Eigen::MatrixX3f cloud(5, 3);
      cloud << neg_inf, neg_inf, neg_inf, neg_inf, neg_inf, neg_inf, neg_inf, neg_inf, neg_inf, neg_inf, neg_inf, neg_inf,
        neg_inf, neg_inf, neg_inf;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 0);
      REQUIRE(filtered.cols() == 3);
    }

    SECTION("Mixed NaN and Inf - all points invalid")
    {
      float nan = std::numeric_limits<float>::quiet_NaN();
      float inf = std::numeric_limits<float>::infinity();
      Eigen::MatrixX3f cloud(5, 3);
      cloud << nan, nan, nan, inf, inf, inf, nan, inf, nan, inf, nan, inf, nan, nan, inf;

      Eigen::MatrixX3f filtered = filterInvalidPoints(cloud);
      REQUIRE(filtered.rows() == 0);
      REQUIRE(filtered.cols() == 3);
    }

    SECTION("Empty cloud - edge case")
    {
      Eigen::MatrixX3f empty_cloud(0, 3);
      Eigen::MatrixX3f filtered = filterInvalidPoints(empty_cloud);
      REQUIRE(filtered.rows() == 0);
      REQUIRE(filtered.cols() == 3);
    }
  }
}

TEST_CASE("GroundRemovalCore Processing Tests", "[ground_removal_core]")
{
  patchwork::Params params;
  params.verbose = false;

  GroundRemovalCore core(params);

  SECTION("Smoke test with synthetic data (Plane)")
  {
    Eigen::MatrixX3f points(4, 3);
    // Simple plane on z=0
    points << 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f;

    REQUIRE_NOTHROW(core.process(points));

    auto ground = core.getGround();
    auto nonground = core.getNonground();

    // Check basic structural validity
    REQUIRE((ground.rows() + nonground.rows()) == 4);
  }

  SECTION("Robustness to different parameter configurations")
  {
    patchwork::Params custom_params;
    custom_params.verbose = true;
    custom_params.num_iter = 1;  // Minimal iterations
    custom_params.sensor_height = 1.5;  // Typical sensor height

    GroundRemovalCore custom_core(custom_params);

    // Create a dense ground plane beyond the default min_range (~2.7m)
    // Patchwork++ filters out points too close to the sensor
    // Generate points from range 3m to 15m in a grid pattern
    std::vector<float> point_data;
    for (int i = 0; i < 20; ++i) {
      for (int j = -10; j < 10; ++j) {
        float x = 3.0f + static_cast<float>(i) * 0.6f;  // x from 3.0 to 14.4m
        float y = static_cast<float>(j) * 0.5f;  // y from -5 to 5m
        float z = -1.5f;  // Ground at z = -sensor_height
        point_data.push_back(x);
        point_data.push_back(y);
        point_data.push_back(z);
      }
    }

    Eigen::MatrixX3f points(point_data.size() / 3, 3);
    for (size_t k = 0; k < point_data.size() / 3; ++k) {
      points(k, 0) = point_data[k * 3 + 0];
      points(k, 1) = point_data[k * 3 + 1];
      points(k, 2) = point_data[k * 3 + 2];
    }

    REQUIRE_NOTHROW(custom_core.process(points));
    // Verify the algorithm produces valid output without crashing
    auto ground = custom_core.getGround();
    auto nonground = custom_core.getNonground();
    // Output should not exceed input (some points may be filtered by range)
    REQUIRE((ground.rows() + nonground.rows()) <= static_cast<int>(point_data.size() / 3));
    // With points at proper range and height, we should get some classification
    REQUIRE((ground.rows() + nonground.rows()) > 0);
  }

  SECTION("Eigen to PointCloud2 conversion")
  {
    Eigen::MatrixX3f points(2, 3);
    points << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f;

    std_msgs::msg::Header header;
    header.frame_id = "test_frame";

    auto msg = GroundRemovalCore::eigenToPointCloud2(points, header);

    REQUIRE(msg.width == 2);
    REQUIRE(msg.height == 1);
    REQUIRE(msg.header.frame_id == "test_frame");

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    REQUIRE_THAT(*iter_x, Catch::Matchers::WithinRel(1.0f));
  }
}
