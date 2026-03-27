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

#include <gtsam/geometry/Point3.h>

#include <cstring>
#include <vector>

#include "eidos/formats/format.hpp"

namespace eidos::formats
{

/**
 * @brief Format for gtsam::Point3 (Eigen::Vector3d).
 * Serializes as 3 packed doubles (24 bytes).
 */
class RawDouble3 : public Format
{
public:
  /**
   * @brief Serialize a gtsam::Point3 to 24 bytes (3 packed doubles: x, y, z).
   * @param data std::any containing a gtsam::Point3.
   * @return 24-byte buffer with packed doubles.
   */
  std::vector<uint8_t> serialize(const std::any & data) override
  {
    auto pt = std::any_cast<gtsam::Point3>(data);
    std::vector<uint8_t> buf(3 * sizeof(double));
    double vals[3] = {pt.x(), pt.y(), pt.z()};
    std::memcpy(buf.data(), vals, buf.size());
    return buf;
  }

  /**
   * @brief Deserialize 24 bytes into a gtsam::Point3.
   * @param bytes Buffer of at least 24 bytes (3 packed doubles).
   * @return std::any containing gtsam::Point3, or empty std::any if undersized.
   */
  std::any deserialize(const std::vector<uint8_t> & bytes) override
  {
    if (bytes.size() < 3 * sizeof(double)) return std::any{};
    double vals[3];
    std::memcpy(vals, bytes.data(), 3 * sizeof(double));
    return gtsam::Point3(vals[0], vals[1], vals[2]);
  }
};

}  // namespace eidos::formats
