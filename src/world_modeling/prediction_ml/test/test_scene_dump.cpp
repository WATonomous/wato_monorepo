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
#include <string>
#include <vector>

#include "prediction_ml/bag_replay.hpp"
#include "prediction_ml/tensor_io.hpp"

#ifndef SCENE_DUMP_EXECUTABLE
#error "SCENE_DUMP_EXECUTABLE must name the scene_dump target"
#endif

namespace
{

std::filesystem::path fixtureRoot()
{
  const char * root = std::getenv("PREDICTION_ML_FIXTURE_DIR");
  return root == nullptr ? std::filesystem::path{"test/fixtures"} :
         std::filesystem::path{root};
}

std::string join(const std::vector<std::string> & values)
{
  std::string result;
  for (const auto & value : values) {
    if (!result.empty()) {
      result += ",";
    }
    result += value;
  }
  return result;
}

std::string quoted(const std::filesystem::path & path)
{
  return "\"" + path.string() + "\"";
}

TEST(SceneDump, SidecarsAndExplicitPinsProduceTheCurrentStepParityBundle)
{
  const auto fixture_dir = fixtureRoot() / "923a0c6bb90f4127";
  const auto pins = prediction_ml::bag_replay::readPinnedTargetIds(fixture_dir);
  ASSERT_EQ(prediction_ml::bag_replay::readReplayStep(fixture_dir), 10U);

  const auto temp_dir = std::filesystem::temp_directory_path();
  const auto sidecar_output = temp_dir / "prediction_ml_scene_dump_sidecar.bin.gz";
  const auto explicit_output = temp_dir / "prediction_ml_scene_dump_explicit.bin.gz";
  std::filesystem::remove(sidecar_output);
  std::filesystem::remove(explicit_output);

  const std::string executable = quoted(SCENE_DUMP_EXECUTABLE);
  const std::string default_command =
    executable + " " + quoted(fixture_dir) + " " + quoted(sidecar_output);
  ASSERT_EQ(std::system(default_command.c_str()), 0);

  const std::string explicit_command =
    executable + " " + quoted(fixture_dir) + " " + quoted(explicit_output) +
    " --pinned " + join(pins);
  ASSERT_EQ(std::system(explicit_command.c_str()), 0);

  const auto sidecar = prediction_ml::readTensorBundle(sidecar_output.string());
  const auto explicit_pins = prediction_ml::readTensorBundle(explicit_output.string());
  const auto expected = prediction_ml::readTensorBundle(
    (fixture_dir / "expected_tensors.bin.gz").string());
  std::filesystem::remove(sidecar_output);
  std::filesystem::remove(explicit_output);

  ASSERT_EQ(sidecar.size(), 8U);
  ASSERT_EQ(explicit_pins.size(), 8U);
  for (const auto & entry : sidecar) {
    const auto & explicit_tensor = explicit_pins.at(entry.first);
    EXPECT_EQ(explicit_tensor.dtype, entry.second.dtype);
    EXPECT_EQ(explicit_tensor.dims, entry.second.dims);
    EXPECT_EQ(explicit_tensor.f32, entry.second.f32);
    EXPECT_EQ(explicit_tensor.u8, entry.second.u8);
    EXPECT_EQ(explicit_tensor.i64, entry.second.i64);

    const auto & expected_tensor = expected.at(entry.first);
    EXPECT_EQ(entry.second.dtype, expected_tensor.dtype);
    EXPECT_EQ(entry.second.dims, expected_tensor.dims);
  }

  EXPECT_EQ(
    sidecar.at("obj_trajs_mask").u8,
    expected.at("obj_trajs_mask").u8);
  EXPECT_EQ(
    sidecar.at("map_polylines_mask").u8,
    expected.at("map_polylines_mask").u8);
  EXPECT_EQ(
    sidecar.at("track_index_to_predict").i64,
    expected.at("track_index_to_predict").i64);
  EXPECT_EQ(
    sidecar.at("center_type_ids").i64,
    expected.at("center_type_ids").i64);
}

}  // namespace
