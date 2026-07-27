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

#include <algorithm>
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

void writeGzip(const std::string & path, const std::vector<uint8_t> & data)
{
  gzFile file = gzopen(path.c_str(), "wb");
  if (file == nullptr) {
    throw formatError(path, "cannot open for writing");
  }

  std::size_t offset = 0U;
  while (offset < data.size()) {
    const std::size_t remaining = data.size() - offset;
    const auto chunk_size = static_cast<unsigned int>(
      std::min(remaining, static_cast<std::size_t>(std::numeric_limits<unsigned int>::max())));
    const int bytes_written = gzwrite(file, data.data() + offset, chunk_size);
    if (bytes_written <= 0) {
      int error_code = Z_OK;
      const char * error_text = gzerror(file, &error_code);
      const std::string detail =
        error_text == nullptr ? "gzip write error" : "gzip write error: " + std::string(error_text);
      gzclose(file);
      throw formatError(path, detail);
    }
    offset += static_cast<std::size_t>(bytes_written);
  }

  if (gzclose(file) != Z_OK) {
    throw formatError(path, "gzip close error");
  }
}

void appendU8(std::vector<uint8_t> & output, const uint8_t value)
{
  output.push_back(value);
}

void appendU32(std::vector<uint8_t> & output, const uint32_t value)
{
  for (std::size_t byte = 0; byte < sizeof(value); ++byte) {
    output.push_back(static_cast<uint8_t>((value >> (8U * byte)) & 0xffU));
  }
}

void appendU64(std::vector<uint8_t> & output, const uint64_t value)
{
  for (std::size_t byte = 0; byte < sizeof(value); ++byte) {
    output.push_back(static_cast<uint8_t>((value >> (8U * byte)) & 0xffU));
  }
}

void appendFloat32(std::vector<uint8_t> & output, const float value)
{
  uint32_t bits = 0U;
  static_assert(sizeof(value) == sizeof(bits), "float32 requires a four-byte float");
  std::memcpy(&bits, &value, sizeof(bits));
  appendU32(output, bits);
}

void appendInt64(std::vector<uint8_t> & output, const int64_t value)
{
  uint64_t bits = 0U;
  static_assert(sizeof(value) == sizeof(bits), "int64 requires an eight-byte integer");
  std::memcpy(&bits, &value, sizeof(bits));
  appendU64(output, bits);
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

void writeTensorBundle(const std::string & path, const TensorBundle & bundle)
{
  if (bundle.size() > std::numeric_limits<uint32_t>::max()) {
    throw formatError(path, "tensor count exceeds uint32");
  }

  std::vector<uint8_t> data;
  data.insert(data.end(), kMagic.begin(), kMagic.end());
  appendU32(data, kVersion);
  appendU32(data, static_cast<uint32_t>(bundle.size()));

  for (const auto & entry : bundle) {
    const auto & name = entry.first;
    const auto & tensor = entry.second;
    if (name.size() > std::numeric_limits<uint32_t>::max()) {
      throw formatError(path, "tensor name is too long");
    }
    if (tensor.dims.size() > std::numeric_limits<uint8_t>::max()) {
      throw formatError(path, "tensor rank exceeds uint8");
    }

    const std::size_t elements = tensor.elementCount();
    std::size_t values = 0U;
    switch (tensor.dtype) {
      case TensorDType::Float32:
        values = tensor.f32.size();
        if (!tensor.u8.empty() || !tensor.i64.empty()) {
          throw formatError(path, "float32 tensor has values in an inactive dtype vector");
        }
        break;
      case TensorDType::UInt8:
        values = tensor.u8.size();
        if (!tensor.f32.empty() || !tensor.i64.empty()) {
          throw formatError(path, "uint8 tensor has values in an inactive dtype vector");
        }
        break;
      case TensorDType::Int64:
        values = tensor.i64.size();
        if (!tensor.f32.empty() || !tensor.u8.empty()) {
          throw formatError(path, "int64 tensor has values in an inactive dtype vector");
        }
        break;
      default:
        throw formatError(path, "unknown dtype code");
    }
    if (values != elements) {
      throw formatError(
              path, "tensor " + name + " has " + std::to_string(values) +
              " values but its shape requires " + std::to_string(elements));
    }

    appendU32(data, static_cast<uint32_t>(name.size()));
    data.insert(data.end(), name.begin(), name.end());
    appendU8(data, static_cast<uint8_t>(tensor.dtype));
    appendU8(data, static_cast<uint8_t>(tensor.dims.size()));
    for (const uint32_t dim : tensor.dims) {
      appendU32(data, dim);
    }

    switch (tensor.dtype) {
      case TensorDType::Float32:
        for (const float value : tensor.f32) {
          appendFloat32(data, value);
        }
        break;
      case TensorDType::UInt8:
        data.insert(data.end(), tensor.u8.begin(), tensor.u8.end());
        break;
      case TensorDType::Int64:
        for (const int64_t value : tensor.i64) {
          appendInt64(data, value);
        }
        break;
      default:
        throw formatError(path, "unknown dtype code");
    }
  }

  writeGzip(path, data);
}

}  // namespace prediction_ml
