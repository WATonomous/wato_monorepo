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
//
// GATE B. Replay committed MCAP fixtures through the real SceneBuilder and
// compare all eight packed tensors against the independent Python reference.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "prediction_ml/scene_builder.hpp"
#include "prediction_ml/tensor_io.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace
{

constexpr float kValueTolerance = 1.0e-4F;
constexpr float kAngleTolerance = 1.0e-5F;
constexpr std::size_t kTargetCapacity = 8U;
constexpr std::size_t kContextCapacity = 128U;
constexpr std::size_t kHistorySteps = 11U;
constexpr std::size_t kObjectFeatureDim = 29U;
constexpr std::size_t kMapPolylineCapacity = 768U;
constexpr std::size_t kPointsPerPolyline = 20U;
constexpr std::size_t kMapFeatureDim = 9U;

constexpr char kTracksTopic[] = "/perception/detections_3D_tracked";
constexpr char kPoseTopic[] = "/localization/pose";
constexpr char kOdomTopic[] = "/world_modeling/transform/odometry";
constexpr char kLaneletTopic[] = "lanelet/lanelet_ahead";

const std::array<std::string, 3> kFixtureScenarios = {
  "262112a66742fe8d",
  "923a0c6bb90f4127",
  "9f62a2970e4e8d4c",
};

std::filesystem::path fixtureRoot()
{
  const char * root = std::getenv("PREDICTION_ML_FIXTURE_DIR");
  return root == nullptr ? std::filesystem::path{"test/fixtures"} : std::filesystem::path{root};
}

std::string trim(std::string value)
{
  const auto not_space = [](const unsigned char character) { return std::isspace(character) == 0; };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::vector<std::string> readPinnedTargetIds(const std::filesystem::path & fixture_dir)
{
  const auto path = fixture_dir / "target_ids.txt";
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("required target pin sidecar is missing: " + path.string());
  }

  std::vector<std::string> ids;
  std::string line;
  while (std::getline(stream, line)) {
    line = trim(std::move(line));
    if (!line.empty()) {
      ids.push_back(std::move(line));
    }
  }
  if (ids.empty()) {
    throw std::runtime_error("refusing to build with an empty target pin list: " + path.string());
  }
  if (ids.size() > kTargetCapacity) {
    ids.resize(kTargetCapacity);
  }
  return ids;
}

std::size_t readReplayStep(const std::filesystem::path & fixture_dir)
{
  const auto path = fixture_dir / "replay_step.txt";
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("required replay-step sidecar is missing: " + path.string());
  }

  long long value = -1;
  stream >> value;
  stream >> std::ws;
  if (!stream || !stream.eof() || value < 0) {
    throw std::runtime_error("invalid non-negative replay step in " + path.string());
  }
  return static_cast<std::size_t>(value);
}

template <typename MessageT>
MessageT deserialize(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  MessageT message;
  rclcpp::Serialization<MessageT> serialization;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

prediction_ml::MtrInputTensors buildFromBag(
  const std::filesystem::path & bag_dir,
  const std::vector<std::string> & pinned_ids,
  const std::size_t replay_step)
{
  if (pinned_ids.empty()) {
    throw std::invalid_argument("buildFromBag requires at least one pinned target id");
  }

  prediction_ml::MtrConfig config;
  config.selected_target_agent_limit = static_cast<int>(kTargetCapacity);
  config.pinned_target_ids = pinned_ids;
  prediction_ml::SceneBuilder builder(config);

  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_dir.string();
  storage_options.storage_id = "mcap";
  reader.open(storage_options);

  prediction_ml::MtrFrameContext frame;
  std::size_t detection_step = 0U;

  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    if (bag_message->topic_name == kPoseTopic) {
      frame.ego_pose = deserialize<geometry_msgs::msg::PoseStamped>(bag_message);
      frame.has_ego = true;
    } else if (bag_message->topic_name == kOdomTopic) {
      const auto odometry = deserialize<nav_msgs::msg::Odometry>(bag_message);
      frame.ego_velocity = odometry.twist.twist;
      frame.has_ego_velocity = true;
    } else if (bag_message->topic_name == kLaneletTopic) {
      frame.lanelet_ahead =
        deserialize<lanelet_msgs::msg::LaneletAhead>(bag_message);
      frame.has_map = true;
    } else if (bag_message->topic_name == kTracksTopic) {
      frame.detections =
        deserialize<vision_msgs::msg::Detection3DArray>(bag_message);
      frame.timestamp =
        static_cast<double>(bag_message->time_stamp) / 1.0e9;
      if (!frame.has_ego || !frame.has_ego_velocity || !frame.has_map) {
        throw std::runtime_error(
                "fixture frame " + std::to_string(detection_step) +
                " is incomplete or not in pose/odom/lanelet/tracks record order");
      }
      builder.addFrame(frame.detections);
      auto result = builder.build(frame);
      if (detection_step == replay_step) {
        return result;
      }
      ++detection_step;
      frame.has_ego = false;
      frame.has_ego_velocity = false;
      frame.has_map = false;
    }
  }

  throw std::runtime_error(
          "replay step " + std::to_string(replay_step) +
          " is outside fixture bag with " + std::to_string(detection_step) +
          " detection frames");
}

void expectBlob(
  const prediction_ml::TensorBundle & bundle,
  const std::string & name,
  const prediction_ml::TensorDType dtype,
  const std::vector<uint32_t> & dims)
{
  const auto found = bundle.find(name);
  ASSERT_NE(found, bundle.end()) << "missing expected tensor " << name;
  EXPECT_EQ(found->second.dtype, dtype) << name << " has the wrong dtype";
  EXPECT_EQ(found->second.dims, dims) << name << " has the wrong shape";
}

void assertEqualMask(
  const std::string & name,
  const std::vector<uint8_t> & actual,
  const std::vector<uint8_t> & expected)
{
  ASSERT_EQ(actual.size(), expected.size()) << name << " size differs";
  std::size_t mismatch_count = 0U;
  std::size_t first_mismatch = 0U;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (actual[i] != expected[i]) {
      if (mismatch_count == 0U) {
        first_mismatch = i;
      }
      ++mismatch_count;
    }
  }
  ASSERT_EQ(mismatch_count, 0U)
    << name << " differs at " << mismatch_count << " of " << expected.size()
    << " elements; first mismatch is flat index " << first_mismatch
    << ". Fix mask parity before interpreting any value differences.";
}

void assertNear(
  const std::string & name,
  const std::vector<float> & actual,
  const std::vector<float> & expected,
  const float tolerance)
{
  ASSERT_EQ(actual.size(), expected.size()) << name << " size differs";
  for (std::size_t i = 0; i < expected.size(); ++i) {
    ASSERT_NEAR(actual[i], expected[i], tolerance)
      << name << " differs at flat index " << i;
  }
}

void assertPaddingIsZero(
  const std::string & name,
  const std::vector<float> & values,
  const std::vector<uint8_t> & mask,
  const std::size_t feature_dim)
{
  ASSERT_EQ(values.size(), mask.size() * feature_dim);
  for (std::size_t slot = 0; slot < mask.size(); ++slot) {
    if (mask[slot] != 0U) {
      continue;
    }
    for (std::size_t feature = 0; feature < feature_dim; ++feature) {
      ASSERT_FLOAT_EQ(values[slot * feature_dim + feature], 0.0F)
        << name << " padding is nonzero at slot " << slot
        << ", feature " << feature;
    }
  }
}

void assertUnusedCentersAreZero(
  const std::string & name,
  const std::vector<float> & centers,
  const std::vector<uint8_t> & point_mask,
  const std::size_t points_per_item)
{
  const std::size_t item_count = point_mask.size() / points_per_item;
  ASSERT_EQ(centers.size(), item_count * 3U);
  for (std::size_t item = 0; item < item_count; ++item) {
    const auto begin = point_mask.begin() + static_cast<std::ptrdiff_t>(item * points_per_item);
    const auto end = begin + static_cast<std::ptrdiff_t>(points_per_item);
    if (std::any_of(begin, end, [](const uint8_t value) { return value != 0U; })) {
      continue;
    }
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      ASSERT_FLOAT_EQ(centers[item * 3U + axis], 0.0F)
        << name << " padding is nonzero at item " << item << ", axis " << axis;
    }
  }
}

class SceneBuilderParity : public ::testing::TestWithParam<std::string>
{
};

TEST_P(SceneBuilderParity, AllInputTensorsMatchPythonReference)
{
  const auto fixture_dir = fixtureRoot() / GetParam();
  const auto expected =
    prediction_ml::readTensorBundle((fixture_dir / "expected_tensors.bin.gz").string());
  const auto pinned_ids = readPinnedTargetIds(fixture_dir);
  const auto replay_step = readReplayStep(fixture_dir);
  const auto actual = buildFromBag(fixture_dir, pinned_ids, replay_step);

  SCOPED_TRACE("fixture " + GetParam());

  // 1. Shapes and dtypes are the fixed model contract.
  ASSERT_EQ(expected.size(), 8U) << "the reference must contain exactly eight input tensors";
  ASSERT_NO_FATAL_FAILURE(expectBlob(
      expected, "obj_trajs", prediction_ml::TensorDType::Float32,
      {kTargetCapacity, kContextCapacity, kHistorySteps, kObjectFeatureDim}));
  ASSERT_NO_FATAL_FAILURE(expectBlob(
      expected, "obj_trajs_mask", prediction_ml::TensorDType::UInt8,
      {kTargetCapacity, kContextCapacity, kHistorySteps}));
  ASSERT_NO_FATAL_FAILURE(expectBlob(
      expected, "obj_trajs_last_pos", prediction_ml::TensorDType::Float32,
      {kTargetCapacity, kContextCapacity, 3U}));
  ASSERT_NO_FATAL_FAILURE(expectBlob(
      expected, "track_index_to_predict", prediction_ml::TensorDType::Int64,
      {kTargetCapacity}));
  ASSERT_NO_FATAL_FAILURE(expectBlob(
      expected, "center_type_ids", prediction_ml::TensorDType::Int64,
      {kTargetCapacity}));
  ASSERT_NO_FATAL_FAILURE(expectBlob(
      expected, "map_polylines", prediction_ml::TensorDType::Float32,
      {kTargetCapacity, kMapPolylineCapacity, kPointsPerPolyline, kMapFeatureDim}));
  ASSERT_NO_FATAL_FAILURE(expectBlob(
      expected, "map_polylines_mask", prediction_ml::TensorDType::UInt8,
      {kTargetCapacity, kMapPolylineCapacity, kPointsPerPolyline}));
  ASSERT_NO_FATAL_FAILURE(expectBlob(
      expected, "map_polylines_center", prediction_ml::TensorDType::Float32,
      {kTargetCapacity, kMapPolylineCapacity, 3U}));

  ASSERT_TRUE(actual.valid) << "SceneBuilder produced no valid tensors";
  ASSERT_EQ(actual.sidecar.targets.size(), pinned_ids.size())
    << "not every explicit target pin was selected";
  for (std::size_t target = 0; target < pinned_ids.size(); ++target) {
    ASSERT_EQ(actual.sidecar.targets[target].detection_id, pinned_ids[target])
      << "target pin order differs at slot " << target;
  }
  ASSERT_EQ(
    actual.obj_trajs_shape,
    (std::vector<int64_t>{
      static_cast<int64_t>(kTargetCapacity),
      static_cast<int64_t>(kContextCapacity),
      static_cast<int64_t>(kHistorySteps),
      static_cast<int64_t>(kObjectFeatureDim)}));
  ASSERT_EQ(
    actual.map_polylines_shape,
    (std::vector<int64_t>{
      static_cast<int64_t>(kTargetCapacity),
      static_cast<int64_t>(kMapPolylineCapacity),
      static_cast<int64_t>(kPointsPerPolyline),
      static_cast<int64_t>(kMapFeatureDim)}));

  // 2. Masks are structural: later comparisons are meaningless until these pass.
  const auto & expected_obj_mask = expected.at("obj_trajs_mask").u8;
  const auto & expected_map_mask = expected.at("map_polylines_mask").u8;
  assertEqualMask("obj_trajs_mask", actual.obj_trajs_mask, expected_obj_mask);
  assertEqualMask("map_polylines_mask", actual.map_polylines_mask, expected_map_mask);

  // 3. Integer metadata must be bit-for-bit identical.
  ASSERT_EQ(
    actual.track_index_to_predict,
    expected.at("track_index_to_predict").i64);
  ASSERT_EQ(actual.center_type_ids, expected.at("center_type_ids").i64);

  // 4. Every masked/padded feature row is exactly zero.
  assertPaddingIsZero(
    "obj_trajs", actual.obj_trajs, actual.obj_trajs_mask, kObjectFeatureDim);
  assertPaddingIsZero(
    "map_polylines", actual.map_polylines, actual.map_polylines_mask,
    kMapFeatureDim);
  assertUnusedCentersAreZero(
    "map_polylines_center", actual.map_polylines_center,
    actual.map_polylines_mask, kPointsPerPolyline);

  // 5. Object fields. Categorical floats are exact; geometry, velocity, and
  // acceleration use 1e-4; sin/cos heading uses 1e-5.
  const auto & expected_obj = expected.at("obj_trajs").f32;
  ASSERT_EQ(actual.obj_trajs.size(), expected_obj.size());
  for (std::size_t slot = 0; slot < expected_obj_mask.size(); ++slot) {
    if (expected_obj_mask[slot] == 0U) {
      continue;
    }
    for (std::size_t feature = 0; feature < kObjectFeatureDim; ++feature) {
      const std::size_t index = slot * kObjectFeatureDim + feature;
      if (feature >= 6U && feature <= 22U) {
        ASSERT_FLOAT_EQ(actual.obj_trajs[index], expected_obj[index])
          << "obj_trajs categorical/time field differs at slot " << slot
          << ", feature " << feature;
      } else {
        const float tolerance =
          (feature == 23U || feature == 24U) ? kAngleTolerance : kValueTolerance;
        ASSERT_NEAR(actual.obj_trajs[index], expected_obj[index], tolerance)
          << "obj_trajs differs at slot " << slot << ", feature " << feature;
      }
    }
  }
  assertNear(
    "obj_trajs_last_pos", actual.obj_trajs_last_pos,
    expected.at("obj_trajs_last_pos").f32, kValueTolerance);

  // 6. Map fields. The semantic token is exact, direction is 1e-5, and
  // positions/pre-positions/centers are 1e-4.
  const auto & expected_map = expected.at("map_polylines").f32;
  ASSERT_EQ(actual.map_polylines.size(), expected_map.size());
  for (std::size_t point = 0; point < expected_map_mask.size(); ++point) {
    if (expected_map_mask[point] == 0U) {
      continue;
    }
    for (std::size_t feature = 0; feature < kMapFeatureDim; ++feature) {
      const std::size_t index = point * kMapFeatureDim + feature;
      if (feature == 6U) {
        ASSERT_FLOAT_EQ(actual.map_polylines[index], expected_map[index])
          << "map global_type differs at point " << point;
      } else {
        const float tolerance =
          (feature >= 3U && feature <= 5U) ? kAngleTolerance : kValueTolerance;
        ASSERT_NEAR(actual.map_polylines[index], expected_map[index], tolerance)
          << "map_polylines differs at point " << point << ", feature " << feature;
      }
    }
  }
  assertNear(
    "map_polylines_center", actual.map_polylines_center,
    expected.at("map_polylines_center").f32, kValueTolerance);
}

INSTANTIATE_TEST_SUITE_P(
  Fixtures,
  SceneBuilderParity,
  ::testing::ValuesIn(kFixtureScenarios),
  [](const auto & info) {return info.param;});

}  // namespace
