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

#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_all.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "patchworkpp/ground_removal_core.hpp"

using wato::perception::patchworkpp::GroundRemovalCore;

namespace
{

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

}  // namespace

TEST_CASE("PointCloud2 conversion contracts", "[ground_removal_core]")
{
  SECTION("Converts xyz fields to Eigen matrix")
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

  SECTION("Throws when required fields are missing")
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.width = 1;
    msg.height = 1;
    auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);

    REQUIRE_THROWS_AS(GroundRemovalCore::pointCloud2ToEigen(msg_ptr), std::runtime_error);
  }
}
