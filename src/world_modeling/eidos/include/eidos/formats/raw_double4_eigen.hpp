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
#include <Eigen/Core>

#include "eidos/formats/format.hpp"

namespace eidos::formats
{

/**
 * @brief Format for Eigen::Vector4d.
 * Serializes as 4 packed doubles (32 bytes).
 */
class RawDouble4Eigen : public Format
{
public:
  std::vector<uint8_t> serialize(const std::any & data) override
  {
    auto vec = std::any_cast<Eigen::Vector4d>(data);
    std::vector<uint8_t> buf(4 * sizeof(double));
    double vals[4] = {vec[0], vec[1], vec[2], vec[3]};
    std::memcpy(buf.data(), vals, buf.size());
    return buf;
  }

  std::any deserialize(const std::vector<uint8_t> & bytes) override
  {
    if (bytes.size() < 4 * sizeof(double)) return std::any{};
    double vals[4];
    std::memcpy(vals, bytes.data(), 4 * sizeof(double));
    return Eigen::Vector4d(vals[0], vals[1], vals[2], vals[3]);
  }
};

}  // namespace eidos::formats
