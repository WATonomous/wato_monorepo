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

#ifndef PREDICTION_ML__TENSOR_IO_HPP_
#define PREDICTION_ML__TENSOR_IO_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace prediction_ml
{

enum class TensorDType : uint8_t
{
  Float32 = 0,
  UInt8 = 1,
  Int64 = 2
};

// One tensor from a bundle. Exactly one value vector is populated, as selected
// by dtype.
struct TensorBlob
{
  TensorDType dtype{TensorDType::Float32};
  std::vector<uint32_t> dims;
  std::vector<float> f32;
  std::vector<uint8_t> u8;
  std::vector<int64_t> i64;

  // Returns the product of dims. A rank-zero tensor is a scalar with one
  // element. Throws std::overflow_error if the product exceeds size_t.
  std::size_t elementCount() const;
};

using TensorBundle = std::map<std::string, TensorBlob>;

// Read a gzipped bundle written by wato_mtr_eval.tensor_io.write_bundle.
// Throws std::runtime_error on a missing file or malformed bundle.
TensorBundle readTensorBundle(const std::string & path);

// Write a bundle in the same little-endian gzip format consumed by
// readTensorBundle and wato_mtr_eval.tensor_io.read_bundle. Throws
// std::runtime_error if a tensor is inconsistent or the file cannot be written.
void writeTensorBundle(const std::string & path, const TensorBundle & bundle);

}  // namespace prediction_ml

#endif  // PREDICTION_ML__TENSOR_IO_HPP_
