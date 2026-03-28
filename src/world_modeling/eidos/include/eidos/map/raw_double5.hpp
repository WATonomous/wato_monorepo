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
#include <cstring>
#include <vector>

#include "eidos/map/format.hpp"

namespace eidos::formats
{

/**
 * @brief Format for std::array<double, 5>.
 * Serializes as 5 packed doubles (40 bytes).
 */
class RawDouble5 : public Format
{
public:
  /**
   * @brief Serialize a std::array<double, 5> to 40 bytes (5 packed doubles).
   * @param data std::any containing a std::array<double, 5>.
   * @return 40-byte buffer with packed doubles.
   */
  std::vector<uint8_t> serialize(const std::any & data) override
  {
    auto arr = std::any_cast<std::array<double, 5>>(data);
    std::vector<uint8_t> buf(5 * sizeof(double));
    std::memcpy(buf.data(), arr.data(), buf.size());
    return buf;
  }

  /**
   * @brief Deserialize 40 bytes into a std::array<double, 5>.
   * @param bytes Buffer of at least 40 bytes (5 packed doubles).
   * @return std::any containing std::array<double, 5>, or empty std::any if undersized.
   */
  std::any deserialize(const std::vector<uint8_t> & bytes) override
  {
    if (bytes.size() < 5 * sizeof(double)) return std::any{};
    std::array<double, 5> arr;
    std::memcpy(arr.data(), bytes.data(), 5 * sizeof(double));
    return arr;
  }
};

}  // namespace eidos::formats
