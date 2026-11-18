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
#include <string>
#include <utility>
#include <vector>

#include "eidos/map/format.hpp"
#include "eidos/utils/visual_frames.hpp"

namespace eidos::formats
{

/**
 * @brief Format for eidos::VisualFrameSet (the compressed camera images of one keyframe).
 *
 * Layout (little-endian): magic "VFS1", set stamp (f64), frame count (u32), then per frame:
 * stamp (f64), frame_id, format and data, each as a u32 length followed by the bytes. The image
 * data stays compressed as it was received.
 */
class VisualFrameSetBinary : public Format
{
public:
  /**
   * @brief Serialize a VisualFrameSet.
   * @param data std::any containing an eidos::VisualFrameSet.
   * @return The packed frame set.
   */
  std::vector<uint8_t> serialize(const std::any & data) override
  {
    const auto & set = std::any_cast<const VisualFrameSet &>(data);
    std::vector<uint8_t> buf;
    std::size_t total = 4 + sizeof(double) + sizeof(uint32_t);
    for (const auto & f : set.frames) {
      total += sizeof(double) + 3 * sizeof(uint32_t) + f->frame_id.size() + f->format.size() + f->data.size();
    }
    buf.reserve(total);
    append(buf, kMagic, 4);
    append(buf, &set.stamp, sizeof(double));
    const auto count = static_cast<uint32_t>(set.frames.size());
    append(buf, &count, sizeof(uint32_t));
    for (const auto & f : set.frames) {
      append(buf, &f->stamp, sizeof(double));
      appendSized(buf, f->frame_id.data(), f->frame_id.size());
      appendSized(buf, f->format.data(), f->format.size());
      appendSized(buf, f->data.data(), f->data.size());
    }
    return buf;
  }

  /**
   * @brief Deserialize a VisualFrameSet.
   * @param bytes Buffer produced by serialize().
   * @return std::any containing eidos::VisualFrameSet, or empty std::any if the buffer is malformed.
   */
  std::any deserialize(const std::vector<uint8_t> & bytes) override
  {
    std::size_t at = 0;
    char magic[4];
    VisualFrameSet set;
    uint32_t count = 0;
    if (!read(bytes, at, magic, 4) || std::memcmp(magic, kMagic, 4) != 0) return std::any{};
    if (!read(bytes, at, &set.stamp, sizeof(double)) || !read(bytes, at, &count, sizeof(uint32_t))) return std::any{};
    for (uint32_t i = 0; i < count; ++i) {
      auto frame = std::make_shared<VisualFrame>();
      std::string data;
      if (
        !read(bytes, at, &frame->stamp, sizeof(double)) || !readSized(bytes, at, frame->frame_id) ||
        !readSized(bytes, at, frame->format) || !readSized(bytes, at, data))
      {
        return std::any{};
      }
      frame->data.assign(data.begin(), data.end());
      set.frames.push_back(std::move(frame));
    }
    return set;
  }

private:
  static constexpr const char * kMagic = "VFS1";

  static void append(std::vector<uint8_t> & buf, const void * src, std::size_t n)
  {
    const auto * p = static_cast<const uint8_t *>(src);
    buf.insert(buf.end(), p, p + n);
  }

  static void appendSized(std::vector<uint8_t> & buf, const void * src, std::size_t n)
  {
    const auto size = static_cast<uint32_t>(n);
    append(buf, &size, sizeof(uint32_t));
    append(buf, src, n);
  }

  static bool read(const std::vector<uint8_t> & bytes, std::size_t & at, void * dst, std::size_t n)
  {
    if (at + n > bytes.size()) return false;
    std::memcpy(dst, bytes.data() + at, n);
    at += n;
    return true;
  }

  static bool readSized(const std::vector<uint8_t> & bytes, std::size_t & at, std::string & out)
  {
    uint32_t size = 0;
    if (!read(bytes, at, &size, sizeof(uint32_t)) || at + size > bytes.size()) return false;
    out.assign(reinterpret_cast<const char *>(bytes.data() + at), size);
    at += size;
    return true;
  }
};

}  // namespace eidos::formats
