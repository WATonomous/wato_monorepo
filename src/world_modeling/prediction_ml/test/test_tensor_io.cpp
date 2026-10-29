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
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
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

TEST(TensorIo, WriterRoundTripsAllSupportedDtypes)
{
  prediction_ml::TensorBundle bundle;

  prediction_ml::TensorBlob floats;
  floats.dtype = prediction_ml::TensorDType::Float32;
  floats.dims = {2U, 3U};
  floats.f32 = {1.0F, -2.5F, 3.25F, 4.0F, 5.0F, 6.0F};
  bundle.emplace("float_values", floats);

  prediction_ml::TensorBlob mask;
  mask.dtype = prediction_ml::TensorDType::UInt8;
  mask.dims = {3U};
  mask.u8 = {0U, 1U, 255U};
  bundle.emplace("mask", mask);

  prediction_ml::TensorBlob index;
  index.dtype = prediction_ml::TensorDType::Int64;
  index.dims = {};
  index.i64 = {-42};
  bundle.emplace("scalar_index", index);

  const auto path =
    std::filesystem::temp_directory_path() / "prediction_ml_tensor_io_round_trip.bin.gz";
  prediction_ml::writeTensorBundle(path.string(), bundle);
  const auto actual = prediction_ml::readTensorBundle(path.string());
  std::filesystem::remove(path);

  ASSERT_EQ(actual.size(), bundle.size());
  for (const auto & expected : bundle) {
    const auto & tensor = actual.at(expected.first);
    EXPECT_EQ(tensor.dtype, expected.second.dtype);
    EXPECT_EQ(tensor.dims, expected.second.dims);
    EXPECT_EQ(tensor.f32, expected.second.f32);
    EXPECT_EQ(tensor.u8, expected.second.u8);
    EXPECT_EQ(tensor.i64, expected.second.i64);
  }
}

TEST(TensorIo, WriterRejectsShapeValueCountMismatch)
{
  prediction_ml::TensorBlob blob;
  blob.dtype = prediction_ml::TensorDType::Float32;
  blob.dims = {2U, 3U};
  blob.f32 = {1.0F};
  prediction_ml::TensorBundle bundle;
  bundle.emplace("bad", std::move(blob));

  const auto path =
    std::filesystem::temp_directory_path() / "prediction_ml_tensor_io_invalid.bin.gz";
  EXPECT_THROW(
    prediction_ml::writeTensorBundle(path.string(), bundle),
    std::runtime_error);
}

TEST(TensorIo, WriterThrowsWhenOutputPathCannotBeOpened)
{
  prediction_ml::TensorBlob blob;
  blob.dtype = prediction_ml::TensorDType::UInt8;
  blob.dims = {1U};
  blob.u8 = {1U};
  prediction_ml::TensorBundle bundle;
  bundle.emplace("x", std::move(blob));

  const auto path = std::filesystem::temp_directory_path() /
    "prediction_ml_missing_output_directory" / "bundle.bin.gz";
  EXPECT_THROW(
    prediction_ml::writeTensorBundle(path.string(), bundle),
    std::runtime_error);
}
