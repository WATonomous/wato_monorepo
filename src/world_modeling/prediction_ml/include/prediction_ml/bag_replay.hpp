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

#ifndef PREDICTION_ML__BAG_REPLAY_HPP_
#define PREDICTION_ML__BAG_REPLAY_HPP_

#include <algorithm>
#include <cctype>
#include <cstddef>
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
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace prediction_ml
{
namespace bag_replay
{

constexpr std::size_t kTargetCapacity = 8U;
constexpr char kTracksTopic[] = "/perception/detections_3D_tracked";
constexpr char kPoseTopic[] = "/localization/pose";
constexpr char kOdomTopic[] = "/world_modeling/transform/odometry";
constexpr char kLaneletTopic[] = "lanelet/lanelet_ahead";

inline std::string trim(std::string value)
{
  const auto not_space = [](const unsigned char character) {
      return std::isspace(character) == 0;
    };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

inline std::vector<std::string> readPinnedTargetIds(
  const std::filesystem::path & fixture_dir)
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

inline std::size_t readReplayStep(const std::filesystem::path & fixture_dir)
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
MessageT deserialize(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  MessageT message;
  rclcpp::Serialization<MessageT> serialization;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

// rosbag2 renamed SerializedBagMessage::time_stamp to recv_timestamp after Humble (Jazzy has only
// the new name). The int/long tag prefers recv_timestamp if a distro ever has both.
template <typename BagMessageT>
auto receiveTimestampNs(const BagMessageT & message, int) -> decltype(message.recv_timestamp)
{
  return message.recv_timestamp;
}

template <typename BagMessageT>
auto receiveTimestampNs(const BagMessageT & message, long) -> decltype(message.time_stamp)  // NOLINT
{
  return message.time_stamp;
}

inline MtrInputTensors buildFromBag(
  const std::filesystem::path & bag_dir,
  const std::vector<std::string> & pinned_ids,
  const std::size_t replay_step)
{
  if (pinned_ids.empty()) {
    throw std::invalid_argument("buildFromBag requires at least one pinned target id");
  }

  MtrConfig config;
  config.selected_target_agent_limit = static_cast<int>(kTargetCapacity);
  config.pinned_target_ids = pinned_ids;
  SceneBuilder builder(config);

  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_dir.string();
  storage_options.storage_id = "mcap";
  reader.open(storage_options);

  MtrFrameContext frame;
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
      frame.lanelet_ahead = deserialize<lanelet_msgs::msg::LaneletAhead>(bag_message);
      frame.has_map = true;
    } else if (bag_message->topic_name == kTracksTopic) {
      frame.detections = deserialize<vision_msgs::msg::Detection3DArray>(bag_message);
      frame.timestamp = static_cast<double>(receiveTimestampNs(*bag_message, 0)) / 1.0e9;
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

inline MtrInputTensors buildFromFixture(const std::filesystem::path & fixture_dir)
{
  return buildFromBag(
    fixture_dir, readPinnedTargetIds(fixture_dir), readReplayStep(fixture_dir));
}

}  // namespace bag_replay
}  // namespace prediction_ml

#endif  // PREDICTION_ML__BAG_REPLAY_HPP_
