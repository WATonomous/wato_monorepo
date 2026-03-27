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

#include <pcl/point_cloud.h>

#include <cstring>
#include <vector>

#include "eidos/formats/format.hpp"
#include "eidos/utils/types.hpp"

namespace eidos::formats
{

/**
 * @brief Format for pcl::PointCloud<PointType>::Ptr.
 *
 * Serializes as raw packed PointType array (no PCD header).
 * Fast memcpy-based read/write.
 */
class PclPcdBinary : public Format
{
public:
  /**
   * @brief Serialize a PCL point cloud to a raw byte buffer.
   * @param data std::any containing a pcl::PointCloud<PointType>::Ptr.
   * @return Packed PointType array as bytes, or empty if the cloud is null/empty.
   */
  std::vector<uint8_t> serialize(const std::any & data) override
  {
    auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
    if (!cloud || cloud->empty()) return {};
    size_t total = cloud->size() * sizeof(PointType);
    std::vector<uint8_t> buf(total);
    std::memcpy(buf.data(), cloud->points.data(), total);
    return buf;
  }

  /**
   * @brief Deserialize a raw byte buffer into a PCL point cloud.
   * @param bytes Packed PointType array previously produced by serialize().
   * @return std::any containing pcl::PointCloud<PointType>::Ptr, or empty
   *         std::any if bytes is empty.
   */
  std::any deserialize(const std::vector<uint8_t> & bytes) override
  {
    if (bytes.empty()) return std::any{};
    size_t num = bytes.size() / sizeof(PointType);
    if (num == 0) return std::any{};
    auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
    cloud->resize(num);
    std::memcpy(cloud->points.data(), bytes.data(), num * sizeof(PointType));
    cloud->width = static_cast<uint32_t>(num);
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }
};

}  // namespace eidos::formats
