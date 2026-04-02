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

#include <cstring>
#include <memory>
#include <vector>

#include <small_gicp/points/point_cloud.hpp>

#include "eidos/map/format.hpp"

namespace eidos::formats
{

/**
 * @brief Format for small_gicp::PointCloud::Ptr.
 *
 * Serializes as: uint64 num_points + packed Eigen::Vector4d array (32 bytes per point).
 * Only stores point positions — normals/covariances are recomputed after deserialization
 * by the consumer if needed (via small_gicp::preprocess_points).
 */
class SmallGicpBinary : public Format
{
public:
  /**
   * @brief Serialize a small_gicp point cloud to bytes.
   *
   * Wire format: uint64_t point count, followed by packed Eigen::Vector4d
   * (32 bytes per point: x, y, z, w).
   *
   * @param data std::any containing a small_gicp::PointCloud::Ptr.
   * @return Byte buffer, or empty if the cloud is null/empty.
   * @note Only positions are serialized. Normals and covariances must be
   *       recomputed after deserialization if needed.
   */
  std::vector<uint8_t> serialize(const std::any & data) override
  {
    auto cloud = std::any_cast<small_gicp::PointCloud::Ptr>(data);
    if (!cloud || cloud->empty()) return {};
    size_t n = cloud->size();
    std::vector<uint8_t> buf(sizeof(uint64_t) + n * 4 * sizeof(double));
    uint64_t num = n;
    std::memcpy(buf.data(), &num, sizeof(uint64_t));
    for (size_t i = 0; i < n; i++) {
      const auto & pt = cloud->point(i);
      double vals[4] = {pt.x(), pt.y(), pt.z(), pt.w()};
      std::memcpy(buf.data() + sizeof(uint64_t) + i * 4 * sizeof(double), vals, 4 * sizeof(double));
    }
    return buf;
  }

  /**
   * @brief Deserialize bytes into a small_gicp point cloud.
   * @param bytes Buffer previously produced by serialize().
   * @return std::any containing small_gicp::PointCloud::Ptr, or empty
   *         std::any if the buffer is too small or malformed.
   */
  std::any deserialize(const std::vector<uint8_t> & bytes) override
  {
    if (bytes.size() < sizeof(uint64_t)) return std::any{};
    uint64_t num;
    std::memcpy(&num, bytes.data(), sizeof(uint64_t));
    if (bytes.size() < sizeof(uint64_t) + num * 4 * sizeof(double)) return std::any{};

    auto cloud = std::make_shared<small_gicp::PointCloud>();
    cloud->resize(num);
    for (uint64_t i = 0; i < num; i++) {
      double vals[4];
      std::memcpy(vals, bytes.data() + sizeof(uint64_t) + i * 4 * sizeof(double), 4 * sizeof(double));
      cloud->point(i) = Eigen::Vector4d(vals[0], vals[1], vals[2], vals[3]);
    }
    return cloud;
  }
};

}  // namespace eidos::formats
