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

#include "prediction_ml/scene_builder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <string>
#include <utility>

namespace prediction_ml
{
namespace
{

constexpr double kUnsetTimestamp = 0.0;

bool isFinite(const double value)
{
  return std::isfinite(value);
}

bool isUsableTimestamp(const double timestamp)
{
  return isFinite(timestamp) && timestamp > kUnsetTimestamp;
}

bool isHistoryConfigValid(const MtrConfig & config)
{
  return config.history_steps > 0 && config.history_rate_hz > 0.0 && isFinite(config.history_rate_hz);
}

std::string lowerAscii(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

double stampToSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

bool isFiniteQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return isFinite(q.x) && isFinite(q.y) && isFinite(q.z) && isFinite(q.w);
}

std::optional<std::array<double, 3>> velocityFromDetection(const vision_msgs::msg::Detection3D & detection)
{
  for (const auto & result : detection.results) {
    if (result.hypothesis.class_id != "linear_velocity" || result.hypothesis.score <= 0.0) {
      continue;
    }

    const auto & velocity = result.pose.pose.position;
    if (!isFinite(velocity.x) || !isFinite(velocity.y) || !isFinite(velocity.z)) {
      return std::nullopt;
    }

    return std::array<double, 3>{velocity.x, velocity.y, velocity.z};
  }

  return std::nullopt;
}

}  // namespace

SceneBuilder::SceneBuilder(MtrConfig config)
: config_(std::move(config)), config_valid_(isHistoryConfigValid(config_))
{}

void SceneBuilder::addFrame(const vision_msgs::msg::Detection3DArray & detections)
{
  const auto timestamp = timestampFromHeader(detections);
  if (!timestamp.has_value()) {
    return;
  }

  addFrameAtTimestamp(detections, *timestamp);
}

MtrInputTensors SceneBuilder::build(const MtrFrameContext & frame)
{
  MtrInputTensors tensors;
  tensors.sidecar.frame_id = frame.detections.header.frame_id;

  if (!config_valid_) {
    return tensors;
  }

  auto current_time = timestampFromHeader(frame.detections);
  if (!current_time.has_value() && isUsableTimestamp(frame.timestamp)) {
    current_time = frame.timestamp;
    addFrameAtTimestamp(frame.detections, *current_time);
  }

  if (!current_time.has_value()) {
    return tensors;
  }

  pruneHistory(*current_time);
  tensors.valid = hasRetainedHistory();
  return tensors;
}

std::optional<double> SceneBuilder::timestampFromHeader(const vision_msgs::msg::Detection3DArray & detections)
{
  const double timestamp = stampToSeconds(detections.header.stamp);
  if (!isUsableTimestamp(timestamp)) {
    return std::nullopt;
  }
  return timestamp;
}

double SceneBuilder::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::optional<SceneBuilder::MtrObjectClass> SceneBuilder::parseObjectClass(
  const vision_msgs::msg::Detection3D & detection)
{
  for (const auto & result : detection.results) {
    const std::string label = lowerAscii(result.hypothesis.class_id);
    if (label == "linear_velocity") {
      continue;
    }

    if (label == "vehicle" || label == "car" || label == "truck" || label == "bus") {
      return MtrObjectClass::Vehicle;
    }
    if (label == "pedestrian" || label == "person") {
      return MtrObjectClass::Pedestrian;
    }
    if (label == "cyclist" || label == "bicycle" || label == "bike") {
      return MtrObjectClass::Cyclist;
    }
  }

  return std::nullopt;
}

std::optional<SceneBuilder::HistorySample> SceneBuilder::sampleFromDetection(
  const vision_msgs::msg::Detection3D & detection, const double timestamp, const std::string & frame_id)
{
  if (detection.id.empty() || !isUsableTimestamp(timestamp)) {
    return std::nullopt;
  }

  const auto object_class = parseObjectClass(detection);
  if (!object_class.has_value()) {
    return std::nullopt;
  }

  const auto & position = detection.bbox.center.position;
  const auto & orientation = detection.bbox.center.orientation;
  const auto & size = detection.bbox.size;

  if (
    !isFinite(position.x) || !isFinite(position.y) || !isFinite(position.z) || !isFiniteQuaternion(orientation) ||
    !isFinite(size.x) || !isFinite(size.y) || !isFinite(size.z) || size.x <= 0.0 || size.y <= 0.0 ||
    size.z <= 0.0) {
    return std::nullopt;
  }

  const double heading = yawFromQuaternion(orientation);
  if (!isFinite(heading)) {
    return std::nullopt;
  }

  HistorySample sample;
  sample.detection_id = detection.id;
  sample.timestamp = timestamp;
  sample.x = position.x;
  sample.y = position.y;
  sample.z = position.z;
  sample.dx = size.x;
  sample.dy = size.y;
  sample.dz = size.z;
  sample.heading = heading;
  sample.object_class = *object_class;
  sample.frame_id = frame_id;

  const auto velocity = velocityFromDetection(detection);
  if (velocity.has_value()) {
    sample.vx = (*velocity)[0];
    sample.vy = (*velocity)[1];
    sample.vz = (*velocity)[2];
  }

  return sample;
}

void SceneBuilder::addFrameAtTimestamp(const vision_msgs::msg::Detection3DArray & detections, const double timestamp)
{
  if (!config_valid_ || !isUsableTimestamp(timestamp)) {
    return;
  }

  for (const auto & detection : detections.detections) {
    const auto sample = sampleFromDetection(detection, timestamp, detections.header.frame_id);
    if (sample.has_value()) {
      addSample(*sample);
    }
  }

  pruneHistory(timestamp);
}

void SceneBuilder::addSample(const HistorySample & sample)
{
  auto it = history_.try_emplace(sample.detection_id).first;
  auto & samples = it->second;
  samples.push_back(sample);
  std::stable_sort(samples.begin(), samples.end(), [](const HistorySample & lhs, const HistorySample & rhs) {
    return lhs.timestamp < rhs.timestamp;
  });
}

void SceneBuilder::pruneHistory(const double current_time)
{
  if (!config_valid_ || !isUsableTimestamp(current_time)) {
    return;
  }

  const double window_length = static_cast<double>(config_.history_steps - 1) / config_.history_rate_hz;
  const double tolerance = 0.5 / config_.history_rate_hz;
  const double oldest_allowed = current_time - window_length - tolerance;

  for (auto it = history_.begin(); it != history_.end();) {
    auto & samples = it->second;
    const auto first_to_keep = std::lower_bound(
      samples.begin(), samples.end(), oldest_allowed, [](const HistorySample & sample, const double timestamp) {
        return sample.timestamp < timestamp;
      });
    samples.erase(samples.begin(), first_to_keep);

    if (samples.empty()) {
      it = history_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<std::string> SceneBuilder::orderedTrackIds() const
{
  std::vector<std::string> ids;
  ids.reserve(history_.size());
  for (const auto & entry : history_) {
    ids.push_back(entry.first);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

bool SceneBuilder::hasRetainedHistory() const
{
  return !history_.empty();
}

}  // namespace prediction_ml
