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

#include "prediction_ml/tensor_io.hpp"

#include <zlib.h>

#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace prediction_ml
{
namespace
{

constexpr std::array<uint8_t, 4> kMagic{{'W', 'M', 'T', 'R'}};
constexpr uint32_t kVersion = 1U;
constexpr std::size_t kChunkSize = 1U << 16U;

std::runtime_error formatError(const std::string & path, const std::string & detail)
{
  return std::runtime_error("tensor bundle " + path + ": " + detail);
}

std::vector<uint8_t> readGzip(const std::string & path)
{
  gzFile file = gzopen(path.c_str(), "rb");
  if (file == nullptr) {
    throw formatError(path, "cannot open");
  }

  std::vector<uint8_t> output;
  std::array<uint8_t, kChunkSize> chunk{};
  while (true) {
    const int bytes_read =
      gzread(file, chunk.data(), static_cast<unsigned int>(chunk.size()));
    if (bytes_read < 0) {
      int error_code = Z_OK;
      const char * error_text = gzerror(file, &error_code);
      const std::string detail =
        error_text == nullptr ? "gzip read error" : "gzip read error: " + std::string(error_text);
      gzclose(file);
      throw formatError(path, detail);
    }
    if (bytes_read == 0) {
      break;
    }
    output.insert(output.end(), chunk.begin(), chunk.begin() + bytes_read);
  }

  const int close_status = gzclose(file);
  if (close_status != Z_OK) {
    throw formatError(path, "truncated or invalid gzip stream");
  }
  return output;
}

class Cursor
{
public:
  explicit Cursor(const std::vector<uint8_t> & data)
  : data_(data) {}

  void require(std::size_t count) const
  {
    if (count > data_.size() - offset_) {
      throw std::runtime_error("tensor bundle: truncated");
    }
  }

  uint8_t readU8()
  {
    require(1U);
    return data_[offset_++];
  }

  uint32_t readU32()
  {
    require(sizeof(uint32_t));
    const uint32_t value =
      static_cast<uint32_t>(data_[offset_]) |
      (static_cast<uint32_t>(data_[offset_ + 1U]) << 8U) |
      (static_cast<uint32_t>(data_[offset_ + 2U]) << 16U) |
      (static_cast<uint32_t>(data_[offset_ + 3U]) << 24U);
    offset_ += sizeof(uint32_t);
    return value;
  }

  uint64_t readU64()
  {
    require(sizeof(uint64_t));
    uint64_t value = 0U;
    for (std::size_t byte = 0; byte < sizeof(uint64_t); ++byte) {
      value |= static_cast<uint64_t>(data_[offset_ + byte]) << (8U * byte);
    }
    offset_ += sizeof(uint64_t);
    return value;
  }

  float readFloat32()
  {
    const uint32_t bits = readU32();
    float value = 0.0F;
    static_assert(sizeof(value) == sizeof(bits), "float32 requires a four-byte float");
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  int64_t readInt64()
  {
    const uint64_t bits = readU64();
    int64_t value = 0;
    static_assert(sizeof(value) == sizeof(bits), "int64 requires an eight-byte integer");
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  std::string readString(std::size_t length)
  {
    require(length);
    const auto * begin = reinterpret_cast<const char *>(data_.data() + offset_);
    std::string value(begin, length);
    offset_ += length;
    return value;
  }

  bool atEnd() const
  {
    return offset_ == data_.size();
  }

private:
  const std::vector<uint8_t> & data_;
  std::size_t offset_{0U};
};

std::size_t checkedByteCount(
  std::size_t elements, std::size_t element_size, const std::string & path)
{
  if (element_size != 0U &&
    elements > std::numeric_limits<std::size_t>::max() / element_size)
  {
    throw formatError(path, "tensor byte count overflows size_t");
  }
  return elements * element_size;
}

}  // namespace

std::size_t TensorBlob::elementCount() const
{
  std::size_t total = 1U;
  for (const uint32_t dim : dims) {
    if (dim != 0U && total > std::numeric_limits<std::size_t>::max() / dim) {
      throw std::overflow_error("tensor dimension product overflows size_t");
    }
    total *= static_cast<std::size_t>(dim);
  }
  return total;
}

TensorBundle readTensorBundle(const std::string & path)
{
  const auto data = readGzip(path);
  Cursor cursor(data);

  for (const uint8_t expected : kMagic) {
    if (cursor.readU8() != expected) {
      throw formatError(path, "bad magic");
    }
  }

  if (cursor.readU32() != kVersion) {
    throw formatError(path, "unsupported version");
  }

  const uint32_t count = cursor.readU32();
  TensorBundle bundle;
  for (uint32_t tensor_index = 0; tensor_index < count; ++tensor_index) {
    const uint32_t name_length = cursor.readU32();
    const std::string name = cursor.readString(name_length);

    const uint8_t dtype_code = cursor.readU8();
    const uint8_t ndim = cursor.readU8();

    TensorBlob tensor;
    tensor.dims.reserve(ndim);
    for (uint8_t dim_index = 0; dim_index < ndim; ++dim_index) {
      tensor.dims.push_back(cursor.readU32());
    }

    const std::size_t elements = tensor.elementCount();
    switch (dtype_code) {
      case static_cast<uint8_t>(TensorDType::Float32):
        tensor.dtype = TensorDType::Float32;
        cursor.require(checkedByteCount(elements, sizeof(float), path));
        tensor.f32.reserve(elements);
        for (std::size_t index = 0; index < elements; ++index) {
          tensor.f32.push_back(cursor.readFloat32());
        }
        break;
      case static_cast<uint8_t>(TensorDType::UInt8):
        tensor.dtype = TensorDType::UInt8;
        cursor.require(checkedByteCount(elements, sizeof(uint8_t), path));
        tensor.u8.reserve(elements);
        for (std::size_t index = 0; index < elements; ++index) {
          tensor.u8.push_back(cursor.readU8());
        }
        break;
      case static_cast<uint8_t>(TensorDType::Int64):
        tensor.dtype = TensorDType::Int64;
        cursor.require(checkedByteCount(elements, sizeof(int64_t), path));
        tensor.i64.reserve(elements);
        for (std::size_t index = 0; index < elements; ++index) {
          tensor.i64.push_back(cursor.readInt64());
        }
        break;
      default:
        throw formatError(path, "unknown dtype code");
    }

    const auto insertion = bundle.emplace(name, std::move(tensor));
    if (!insertion.second) {
      throw formatError(path, "duplicate tensor name: " + name);
    }
  }

  if (!cursor.atEnd()) {
    throw formatError(path, "trailing data");
  }
  return bundle;
}

}  // namespace prediction_ml
