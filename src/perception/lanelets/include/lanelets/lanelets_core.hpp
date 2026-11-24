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

// A point in the rear axle (ego) frame: origin at the rear axle, x forward, y left, in meters.
struct EgoPoint
{
  float x;
  float y;
};

class LaneletsCore
{
public:
  static constexpr int kLidarFeatures = 5;

  // How far ahead of the rear axle the origin of SemVecNet's output frame sits, in meters.
  // This is a property of the trained model (a virtual nuScenes LIDAR_TOP), NOT of Eve's physical
  // lidar, which eve_description places 0.8629 m ahead of rear_axle. Do not derive it from TF.
  // Keep in sync with the model's source of truth:
  //   wato-SemVecNet: SemVecNet/projects/mmdet3d_plugin/datasets/ringroad_augmentation.py (0.943713)
  //   wato-SemVecNet-cpp: CUDA-SemVecNet/src/main.cpp, kLidarForwardOffset
  static constexpr float kModelOriginForwardOffset = 0.943713f;

  LaneletsCore(int image_width, int image_height, int max_points);

  bool validateAndNormalizeImage(cv::Mat & rgb_image) const;
  // Packs finite x/y/z/intensity/time_lag records as IEEE binary16 bits, limited to max_points.
  std::vector<std::uint16_t> processLidar(const sensor_msgs::msg::PointCloud2 & cloud) const;
  static CameraCalibration computeCameraCalibration(
    const sensor_msgs::msg::CameraInfo & info, const geometry_msgs::msg::TransformStamped & lidar_to_camera,
    int input_width, int input_height);
  static Matrix4 flattenTransform(const geometry_msgs::msg::TransformStamped & transform);

  // SemVecNet model frame (origin at the lidar, x right, y forward) -> rear axle frame.
  static EgoPoint toRearAxle(float model_x_right, float model_y_forward);

  // Converts every point of a polyline. `model_points` is any container of {x, y} pairs, such as
  // semvecnet::postprocessing::Polyline::points (a template, so the SemVecNet type does not have to
  // appear in this public header). Returns the same container type, with each pair replaced by the
  // rear axle {x forward, y left}. Use it to overwrite a polyline's points in place:
  //   polyline.points = LaneletsCore::toRearAxlePoints(polyline.points);
  template <typename Points>
  static Points toRearAxlePoints(const Points & model_points)
  {
    Points ego_points = model_points;
    for (auto & point : ego_points) {
      const auto ego = toRearAxle(point[0], point[1]);
      point[0] = ego.x;
      point[1] = ego.y;
    }
    return ego_points;
  }

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
