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

#include <gtest/gtest.h>

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include "prediction_ml/tensor_io.hpp"

namespace
{

std::string fixturePath(const std::string & name)
{
  const char * root = std::getenv("PREDICTION_ML_FIXTURE_DIR");
  return std::string(root ? root : "test/fixtures") + "/" + name;
}

}  // namespace

TEST(TensorIo, ReadsFloatTensorWithCorrectShapeAndValues)
{
  const auto bundle = prediction_ml::readTensorBundle(fixturePath("tensor_io_sample.bin.gz"));
  ASSERT_EQ(bundle.count("obj_trajs"), 1U);

  const auto & blob = bundle.at("obj_trajs");
  EXPECT_EQ(blob.dtype, prediction_ml::TensorDType::Float32);
  EXPECT_EQ(blob.dims, (std::vector<uint32_t>{2, 3, 4, 5}));
  EXPECT_EQ(blob.f32.size(), 2U * 3U * 4U * 5U);
  EXPECT_EQ(blob.elementCount(), 120U);
}

TEST(TensorIo, ReadsMaskAsUint8AndIndexAsInt64)
{
  const auto bundle = prediction_ml::readTensorBundle(fixturePath("tensor_io_sample.bin.gz"));

  const auto & mask = bundle.at("obj_trajs_mask");
  EXPECT_EQ(mask.dtype, prediction_ml::TensorDType::UInt8);
  EXPECT_EQ(mask.u8.size(), 2U * 3U * 4U);

  const auto & index = bundle.at("track_index_to_predict");
  EXPECT_EQ(index.dtype, prediction_ml::TensorDType::Int64);
  EXPECT_EQ(index.i64, (std::vector<int64_t>{0, 1}));
}

TEST(TensorIo, RoundTripsValuesWrittenByPython)
{
  const auto bundle = prediction_ml::readTensorBundle(fixturePath("tensor_io_ordered.bin.gz"));
  const auto & blob = bundle.at("ordered");
  ASSERT_EQ(blob.f32.size(), 120U);
  for (std::size_t i = 0; i < blob.f32.size(); ++i) {
    EXPECT_FLOAT_EQ(blob.f32[i], static_cast<float>(i)) << "at flat index " << i;
  }
}

TEST(TensorIo, ReadsSingleElementTensor)
{
  const auto bundle = prediction_ml::readTensorBundle(fixturePath("tensor_io_scalar.bin.gz"));
  const auto & blob = bundle.at("scalar");
  EXPECT_EQ(blob.dims, (std::vector<uint32_t>{1}));
  EXPECT_EQ(blob.elementCount(), 1U);
  EXPECT_EQ(blob.i64, (std::vector<int64_t>{42}));
}

TEST(TensorIo, ThrowsOnMissingFile)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("no_such_file.bin.gz")),
    std::runtime_error);
}

TEST(TensorIo, ThrowsOnBadMagic)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("tensor_io_bad_magic.bin.gz")),
    std::runtime_error);
}

TEST(TensorIo, ThrowsOnUnsupportedVersion)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("tensor_io_bad_version.bin.gz")),
    std::runtime_error);
}

TEST(TensorIo, ThrowsOnUnknownDtype)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("tensor_io_bad_dtype.bin.gz")),
    std::runtime_error);
}

TEST(TensorIo, ThrowsOnTruncatedPayload)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("tensor_io_truncated.bin.gz")),
    std::runtime_error);
}

TEST(TensorIo, ThrowsOnTruncatedGzipStream)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("tensor_io_truncated_gzip.bin.gz")),
    std::runtime_error);
}

TEST(TensorIo, ThrowsOnOverflowingDimensions)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("tensor_io_overflow.bin.gz")),
    std::runtime_error);
}

TEST(TensorIo, ThrowsOnDuplicateTensorName)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("tensor_io_duplicate.bin.gz")),
    std::runtime_error);
}

TEST(TensorIo, ThrowsOnTrailingData)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("tensor_io_trailing.bin.gz")),
    std::runtime_error);
}
