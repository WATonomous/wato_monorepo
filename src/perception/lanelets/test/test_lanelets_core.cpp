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

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "lanelets/lanelets_core.hpp"

namespace wato::perception::lanelets
{
namespace
{

TEST(LaneletsCore, PreparesPackedRgbWithoutChangingChannelOrder)
{
  const cv::Vec3b pixel(10, 20, 30);
  const cv::Mat source(4, 6, CV_8UC3, cv::Scalar(10, 20, 30));
  auto cropped = source(cv::Rect(1, 1, 3, 2));
  ASSERT_FALSE(cropped.isContinuous());
  EXPECT_TRUE(LaneletsCore(3, 2, 1).validateAndNormalizeImage(cropped));
  EXPECT_TRUE(cropped.isContinuous());
  EXPECT_EQ(cropped.at<cv::Vec3b>(0, 0), pixel);
  EXPECT_TRUE(LaneletsCore(4, 5, 1).validateAndNormalizeImage(cropped));
  EXPECT_EQ(cropped.size(), cv::Size(4, 5));
  EXPECT_EQ(cropped.at<cv::Vec3b>(4, 3), pixel);
}

TEST(LaneletsCore, PacksFiveHalfFeaturesAndSkipsInvalidPoints)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.width = 1;
  cloud.height = 3;
  cloud.point_step = 20;
  cloud.row_step = 24;
  cloud.data.resize(cloud.row_step * cloud.height);
  for (const char * name : {"x", "y", "z", "intensity", "ring"}) {
    auto & field = cloud.fields.emplace_back();
    field.name = name;
    field.offset = (cloud.fields.size() - 1) * sizeof(float);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
  }
  const float invalid[5]{std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, 0};
  const float valid[5]{1.5f, -2.0f, 3.0f, 12.0f, 31.0f};
  std::memcpy(cloud.data.data(), invalid, sizeof(invalid));
  std::memcpy(cloud.data.data() + cloud.row_step, valid, sizeof(valid));
  std::memcpy(cloud.data.data() + 2 * cloud.row_step, valid, sizeof(valid));
  const LaneletsCore core(1, 1, 1);
  const auto points = core.processLidar(cloud);
  ASSERT_EQ(points.size(), 5u);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(static_cast<float>(cv::float16_t::fromBits(points[i])), valid[i]);
  }
  EXPECT_EQ(points[4], 0u);
  EXPECT_EQ(points[0], 0x3e00u);  // IEEE binary16 encoding of 1.5.
  cloud.data.pop_back();
  EXPECT_THROW(core.processLidar(cloud), std::invalid_argument);
}

TEST(LaneletsCore, ComputesMaskSpaceIntrinsicsAndRowMajorTransform)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 1280;
  info.height = 1024;
  info.p = {800, 0, 640, 0, 0, 1024, 512, 0, 0, 0, 1, 0};
  info.r = {0, -1, 0, 1, 0, 0, 0, 0, 1};
  geometry_msgs::msg::TransformStamped tf;
  tf.transform.rotation.w = 1.0;
  tf.transform.translation.x = 2.0;
  tf.transform.translation.y = 3.0;
  const auto calibration = LaneletsCore::computeCameraCalibration(info, tf);
  const std::array<float, 9> expected_intrinsics{600, 0, 480, 0, 720, 360, 0, 0, 1};
  EXPECT_EQ(calibration.intrinsics, expected_intrinsics);
  const Matrix4 expected_camera{0, -1, 0, -3, 1, 0, 0, 2, 0, 0, 1, 0, 0, 0, 0, 1};
  EXPECT_EQ(calibration.lidar_to_camera, expected_camera);
  const Matrix4 expected_map{1, 0, 0, 2, 0, 1, 0, 3, 0, 0, 1, 0, 0, 0, 0, 1};
  EXPECT_EQ(LaneletsCore::flattenTransform(tf), expected_map);
  info.r = {};
  EXPECT_THROW(LaneletsCore::computeCameraCalibration(info, tf), std::invalid_argument);
}

TEST(LaneletsCore, ConvertsLanelet2GeometryToRosMessage)
{
  const lanelet::LineString3d left(
    10, {lanelet::Point3d(1, 0.0, 2.0, 1.0), lanelet::Point3d(2, 10.0, 2.0, 1.0)});
  const lanelet::LineString3d right(
    11, {lanelet::Point3d(3, 0.0, -2.0, 1.0), lanelet::Point3d(4, 10.0, -2.0, 1.0)});
  const lanelet::Lanelet detected(12, left, right);
  const auto msg = LaneletsCore::toLaneletMsg(detected);

  EXPECT_EQ(msg.id, 12);
  ASSERT_EQ(msg.left_boundary.size(), 2u);
  ASSERT_EQ(msg.right_boundary.size(), 2u);
  EXPECT_DOUBLE_EQ(msg.left_boundary.front().y, 2.0);
  EXPECT_DOUBLE_EQ(msg.right_boundary.back().y, -2.0);
  ASSERT_EQ(msg.centerline.size(), detected.centerline().size());
  ASSERT_FALSE(msg.centerline.empty());
  EXPECT_DOUBLE_EQ(msg.centerline.front().y, 0.0);
  EXPECT_DOUBLE_EQ(msg.centerline.back().x, 10.0);
  EXPECT_DOUBLE_EQ(msg.centerline.back().z, 1.0);
  EXPECT_EQ(msg.left_lane_id, -1);
  EXPECT_FALSE(msg.can_change_left);
}

}  // namespace
}  // namespace wato::perception::lanelets
