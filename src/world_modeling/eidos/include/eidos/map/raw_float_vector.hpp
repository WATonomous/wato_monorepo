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
#include <vector>

#include "eidos/map/format.hpp"

namespace eidos::formats
{

/**
 * @brief Format for std::vector<float> (e.g. image descriptors).
 * Serializes as the packed floats; the element count is the blob size / 4.
 */
class RawFloatVector : public Format
{
public:
  /**
   * @brief Serialize a std::vector<float> to its packed bytes.
   * @param data std::any containing a std::vector<float>.
   * @return 4 * size bytes.
   */
  std::vector<uint8_t> serialize(const std::any & data) override
  {
    const auto & vec = std::any_cast<const std::vector<float> &>(data);
    std::vector<uint8_t> buf(vec.size() * sizeof(float));
    if (!buf.empty()) std::memcpy(buf.data(), vec.data(), buf.size());
    return buf;
  }

  /**
   * @brief Deserialize packed floats into a std::vector<float>.
   * @param bytes Buffer whose size is a multiple of 4.
   * @return std::any containing std::vector<float>, or empty std::any if the size is not a multiple of 4.
   */
  std::any deserialize(const std::vector<uint8_t> & bytes) override
  {
    if (bytes.size() % sizeof(float) != 0) return std::any{};
    std::vector<float> vec(bytes.size() / sizeof(float));
    if (!vec.empty()) std::memcpy(vec.data(), bytes.data(), bytes.size());
    return vec;
  }
};

}  // namespace eidos::formats
