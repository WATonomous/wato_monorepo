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

#include <any>
#include <cstdint>
#include <vector>

namespace eidos::formats
{

/**
 * @brief Base class for data format handlers.
 *
 * Each format knows the concrete C++ type it handles and provides:
 * - serialize: std::any → bytes (for SQLite persistence)
 * - deserialize: bytes → std::any (from SQLite persistence)
 *
 * The std::any must contain the correct C++ type for this format.
 * A type mismatch throws std::bad_any_cast during serialize.
 *
 * MapManager's store<T>() / retrieve<T>() handle the T ↔ std::any
 * conversion via templates. Formats handle std::any ↔ bytes.
 */
class Format
{
public:
  virtual ~Format() = default;

  /// Serialize data from std::any to byte buffer.
  /// Throws std::bad_any_cast if the std::any contains the wrong type.
  virtual std::vector<uint8_t> serialize(const std::any & data) = 0;

  /// Deserialize byte buffer to std::any containing the correct C++ type.
  virtual std::any deserialize(const std::vector<uint8_t> & bytes) = 0;
};

}  // namespace eidos::formats
