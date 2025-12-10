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

#include <vector>
#include <string>
#include <limits>

#include <catch2/catch_all.hpp>
#include <wato_test/wato_test.hpp>

#include "patchworkpp/ground_removal_core.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using wato::perception::patchworkpp::GroundRemovalCore;

// Helper to create a simple PointCloud2
sensor_msgs::msg::PointCloud2 createPointCloud(
  size_t width, size_t height, const std::vector<float>& points_xyz)
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
    ++iter_x; ++iter_y; ++iter_z;
  }
  return msg;
}

TEST_CASE("GroundRemovalCore Conversion Tests", "[ground_removal_core]")
{
  SECTION("PointCloud2 to Eigen conversion (Valid Data)")
  {
    std::vector<float> points = {
      1.0f, 2.0f, 3.0f,
      4.0f, 5.0f, 6.0f
    };
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
    std::vector<float> points = {
      nan, 2.0f, 3.0f,
      1.0f, inf, 3.0f,
      1.0f, 2.0f, -inf
    };
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

  SECTION("Throws on unreasonable dimensions (Protection Limit)")
  {
    // Mock a huge point cloud that exceeds the internal safety limits
    // defined in GroundRemovalCore (max_width=1000000, max_height=10000, max_total=10000000)
    sensor_msgs::msg::PointCloud2 msg;
    msg.width = 1000001; 
    msg.height = 1;
    msg.data.resize(1); // Minimal data to avoid bad alloc, the check happens before data read
    auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);

    REQUIRE_THROWS_AS(GroundRemovalCore::pointCloud2ToEigen(msg_ptr), std::runtime_error);
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
    points << 0.0f, 0.0f, 0.0f,
              1.0f, 0.0f, 0.0f,
              0.0f, 1.0f, 0.0f,
              1.0f, 1.0f, 0.0f;

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
    custom_params.num_iter = 1; // Minimal iterations
    custom_params.sensor_height = 1.5; // Typical sensor height

    GroundRemovalCore custom_core(custom_params);
    
    // Create a dense ground plane beyond the default min_range (~2.7m)
    // Patchwork++ filters out points too close to the sensor
    // Generate points from range 3m to 15m in a grid pattern
    std::vector<float> point_data;
    for (int i = 0; i < 20; ++i) {
      for (int j = -10; j < 10; ++j) {
        float x = 3.0f + static_cast<float>(i) * 0.6f;  // x from 3.0 to 14.4m
        float y = static_cast<float>(j) * 0.5f;          // y from -5 to 5m
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
    points << 1.0f, 2.0f, 3.0f,
              4.0f, 5.0f, 6.0f;

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
