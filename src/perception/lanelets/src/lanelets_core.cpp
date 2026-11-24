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

#include "lanelets/lanelets_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace wato::perception::lanelets
{

  namespace
  {
    constexpr float kMaxFp16 = 65504.0f;
    using PointCloud = sensor_msgs::msg::PointCloud2;
    using PointField = sensor_msgs::msg::PointField;

    std::size_t floatFieldOffset(const PointCloud &cloud, const char *name)
    {
      const auto field =
          std::find_if(cloud.fields.begin(), cloud.fields.end(), [name](const auto &f)
                       { return f.name == name; });
      if (
          field == cloud.fields.end() || field->datatype != PointField::FLOAT32 || field->count != 1 ||
          cloud.point_step < sizeof(float) || field->offset > cloud.point_step - sizeof(float))
      {
        throw std::invalid_argument(std::string("Expected a FLOAT32 LiDAR field named ") + name);
      }
      return field->offset;
    }

    Matrix4 flattenMatrix(const Eigen::Matrix4d &matrix)
    {
      Matrix4 result;
      Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> flat(result.data());
      flat = matrix.cast<float>();
      return result;
    }
  } // namespace

  LaneletsCore::LaneletsCore(int image_width, int image_height, int max_points)
      : image_width_(image_width), image_height_(image_height), max_points_(max_points)
  {
    if (image_width_ <= 0 || image_height_ <= 0 || max_points_ <= 0)
    {
      throw std::invalid_argument("Expected positive image dimensions and max_points");
    }
  }

  bool LaneletsCore::validateAndNormalizeImage(cv::Mat &rgb_image) const
  {
    // The ROS decoders already ensure RGB8; all cameras must have the same packed layout.
    if (rgb_image.empty() || rgb_image.type() != CV_8UC3)
    {
      return false;
    }
    if (rgb_image.cols != image_width_ || rgb_image.rows != image_height_)
    {
      cv::resize(rgb_image, rgb_image, cv::Size(image_width_, image_height_), 0, 0, cv::INTER_LINEAR);
    }
    else if (!rgb_image.isContinuous())
    {
      rgb_image = rgb_image.clone();
    }
    return true;
  }

  std::vector<std::uint16_t> LaneletsCore::processLidar(const PointCloud &cloud) const
  {
    if (cloud.width == 0 || cloud.height == 0)
    {
      return {};
    }
    if (
        static_cast<std::size_t>(cloud.width) * cloud.point_step > cloud.row_step ||
        cloud.height > cloud.data.size() / std::max(1u, cloud.row_step))
    {
      throw std::invalid_argument("Invalid PointCloud2 dimensions or strides");
    }

    const std::array<std::size_t, 4> offsets{
        floatFieldOffset(cloud, "x"),
        floatFieldOffset(cloud, "y"),
        floatFieldOffset(cloud, "z"),
        floatFieldOffset(cloud, "intensity")};
    const uint16_t endian_probe = 1;
    const bool host_big_endian = *reinterpret_cast<const unsigned char *>(&endian_probe) == 0;
    std::vector<std::uint16_t> lidar_data;
    const auto point_limit = static_cast<std::size_t>(max_points_) * kLidarFeatures;
    lidar_data.reserve(std::min(point_limit, static_cast<std::size_t>(cloud.width) * cloud.height * kLidarFeatures));
    // Walk by row/point strides so organized clouds and unaligned fields work too.
    for (uint32_t row = 0; row < cloud.height; ++row)
    {
      for (uint32_t col = 0; col < cloud.width; ++col)
      {
        const auto *point = cloud.data.data() + std::size_t{row} * cloud.row_step + std::size_t{col} * cloud.point_step;
        std::array<float, kLidarFeatures> values{}; // Current sweep time lag is zero (not the ring index).
        for (std::size_t i = 0; i < offsets.size(); ++i)
        {
          unsigned char bytes[sizeof(float)];
          std::memcpy(bytes, point + offsets[i], sizeof(float));
          if (cloud.is_bigendian != host_big_endian)
          {
            std::reverse(std::begin(bytes), std::end(bytes));
          }
          std::memcpy(&values[i], bytes, sizeof(float));
        }
        if (!std::all_of(values.begin(), values.end(), [](float v)
                         { return std::abs(v) <= kMaxFp16; }))
        {
          continue;
        }
        // Same FP16 point layout as BEVFusion, without a CUDA header or runtime dependency.
        for (const float value : values)
        {
          lidar_data.push_back(cv::float16_t(value).bits());
        }
        if (lidar_data.size() == point_limit)
        {
          return lidar_data;
        }
      }
    }
    return lidar_data;
  }

  CameraCalibration LaneletsCore::computeCameraCalibration(
      const sensor_msgs::msg::CameraInfo &info, const geometry_msgs::msg::TransformStamped &lidar_to_camera,
      int input_width, int input_height)
  {
    // CameraInfo P projects onto image_rect; K describes the distorted image.
    // The inputs here are monocular streams, so P has no stereo baseline.
    const auto finite = [](double v)
    { return std::isfinite(v); };
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(info.r.data());
    if (
        input_width <= 0 || input_height <= 0 || info.width == 0 || info.height == 0 ||
        !std::all_of(info.p.begin(), info.p.end(), finite) || info.p[0] <= 0.0 ||
        info.p[5] <= 0.0 || info.p[3] != 0.0 || info.p[7] != 0.0 || info.p[11] != 0.0 || !rotation.allFinite() ||
        std::abs(rotation.determinant() - 1.0) > 1e-3)
    {
      throw std::invalid_argument("Expected calibrated, rectified monocular CameraInfo");
    }
    CameraCalibration calibration;
    // SemVecNet handles its own input-to-HRNet mask scaling.
    const double sx = static_cast<double>(input_width) / info.width;
    const double sy = static_cast<double>(input_height) / info.height;
    calibration.intrinsics = {
        static_cast<float>(sx * info.p[0]),
        static_cast<float>(sx * info.p[1]),
        static_cast<float>(sx * info.p[2]),
        static_cast<float>(sy * info.p[4]),
        static_cast<float>(sy * info.p[5]),
        static_cast<float>(sy * info.p[6]),
        static_cast<float>(info.p[8]),
        static_cast<float>(info.p[9]),
        static_cast<float>(info.p[10])};

    Eigen::Matrix4d rectification = Eigen::Matrix4d::Identity();
    rectification.topLeftCorner<3, 3>() = rotation;
    calibration.lidar_to_camera = flattenMatrix(rectification * tf2::transformToEigen(lidar_to_camera).matrix());
    return calibration;
  }

  Matrix4 LaneletsCore::flattenTransform(const geometry_msgs::msg::TransformStamped &transform)
  {
    return flattenMatrix(tf2::transformToEigen(transform).matrix());
  }

  lanelet_msgs::msg::Lanelet LaneletsCore::toLaneletMsg(const lanelet::ConstLanelet &lanelet)
  {
    lanelet_msgs::msg::Lanelet msg;
    msg.id = lanelet.id();
    const auto copy_points = [](const auto &line, auto &points)
    {
      points.reserve(line.size());
      for (const auto &point : line)
      {
        auto &out = points.emplace_back();
        out.x = point.x();
        out.y = point.y();
        out.z = point.z();
      }
    };
    copy_points(lanelet.leftBound(), msg.left_boundary);
    copy_points(lanelet.rightBound(), msg.right_boundary);
    copy_points(lanelet.centerline(), msg.centerline);

    // Geometry only. Do not imply dashed markings or lane-change permissions.
    msg.left_boundary_type = msg.BOUNDARY_SOLID;
    msg.right_boundary_type = msg.BOUNDARY_SOLID;
    msg.left_lane_id = -1;
    msg.right_lane_id = -1;
    return msg;
  }

} // namespace wato::perception::lanelets
