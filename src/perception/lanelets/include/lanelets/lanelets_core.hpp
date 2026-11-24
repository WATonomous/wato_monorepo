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

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet_msgs/msg/lanelet.hpp>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace wato::perception::lanelets
{

using Matrix4 = std::array<float, 16>;

// Same fields and layout convention as SemVecNet's CameraCalibration.
struct CameraCalibration
{
  std::array<float, 9> intrinsics;  // Row-major, in the input image's pixel coordinates.
  Matrix4 lidar_to_camera;  // Row-major, into the rectified optical frame.
};

class LaneletsCore
{
public:
  static constexpr int kLidarFeatures = 5;

  LaneletsCore(int image_width, int image_height, int max_points);

  bool validateAndNormalizeImage(cv::Mat & rgb_image) const;
  // Packs finite x/y/z/intensity/time_lag records as IEEE binary16 bits, limited to max_points.
  std::vector<std::uint16_t> processLidar(const sensor_msgs::msg::PointCloud2 & cloud) const;
  static CameraCalibration computeCameraCalibration(
    const sensor_msgs::msg::CameraInfo & info, const geometry_msgs::msg::TransformStamped & lidar_to_camera,
    int input_width, int input_height);
  static Matrix4 flattenTransform(const geometry_msgs::msg::TransformStamped & transform);
  static lanelet_msgs::msg::Lanelet toLaneletMsg(const lanelet::ConstLanelet & lanelet);

  int imageWidth() const
  {
    return image_width_;
  }

  int imageHeight() const
  {
    return image_height_;
  }

private:
  int image_width_;
  int image_height_;
  int max_points_;
};

}  // namespace wato::perception::lanelets
