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
#include <cstdint>
#include <iterator>
#include <string>
#include <utility>

namespace prediction_ml
{
namespace
{

constexpr double kUnsetTimestamp = 0.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

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
  MtrInputTensors tensors{};
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

  if (!frame.has_ego) {
    return tensors;
  }

  const auto targets = selectTargets(frame.detections, *current_time, frame.ego_pose);
  if (targets.empty()) {
    return tensors;
  }

  const auto context_track_ids = orderedTrackIds();
  if (context_track_ids.empty()) {
    return tensors;
  }

  const auto desired_times = desiredSampleTimes(*current_time);
  const auto resampled_tracks = resampleTracks(context_track_ids, desired_times);
  fillTargetContextOutputs(targets, context_track_ids, resampled_tracks, tensors);
  tensors.valid = !tensors.sidecar.targets.empty();
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

double SceneBuilder::clampForwardHalfAngleDeg(const double half_angle_deg)
{
  if (!isFinite(half_angle_deg) || half_angle_deg < 0.0) {
    return 0.0;
  }
  if (half_angle_deg > 180.0) {
    return 180.0;
  }
  return half_angle_deg;
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

std::vector<double> SceneBuilder::desiredSampleTimes(const double current_time) const
{
  std::vector<double> times;
  if (!config_valid_ || !isUsableTimestamp(current_time)) {
    return times;
  }

  times.reserve(static_cast<std::size_t>(config_.history_steps));
  for (int i = 0; i < config_.history_steps; ++i) {
    const double steps_from_current = static_cast<double>(config_.history_steps - 1 - i);
    times.push_back(current_time - steps_from_current / config_.history_rate_hz);
  }
  return times;
}

std::optional<SceneBuilder::HistorySample> SceneBuilder::nearestSampleForSlot(
  const std::vector<HistorySample> & samples, const double desired_time) const
{
  if (samples.empty() || !isUsableTimestamp(desired_time)) {
    return std::nullopt;
  }

  const double tolerance = 0.5 / config_.history_rate_hz;
  const auto first_after = std::lower_bound(
    samples.begin(), samples.end(), desired_time, [](const HistorySample & sample, const double timestamp) {
      return sample.timestamp < timestamp;
    });

  const HistorySample * best = nullptr;
  double best_delta = 0.0;

  const auto consider = [&](const HistorySample & sample) {
    const double delta = std::abs(sample.timestamp - desired_time);
    if (delta > tolerance) {
      return;
    }
    if (best == nullptr || delta < best_delta || (delta == best_delta && sample.timestamp > best->timestamp)) {
      best = &sample;
      best_delta = delta;
    }
  };

  if (first_after != samples.end()) {
    consider(*first_after);
  }
  if (first_after != samples.begin()) {
    consider(*std::prev(first_after));
  }

  if (best == nullptr) {
    return std::nullopt;
  }
  return *best;
}

std::vector<SceneBuilder::ResampledTrack> SceneBuilder::resampleTracks(
  const std::vector<std::string> & track_ids, const std::vector<double> & desired_times) const
{
  std::vector<ResampledTrack> tracks;
  tracks.reserve(track_ids.size());

  for (const auto & track_id : track_ids) {
    const auto history_it = history_.find(track_id);
    if (history_it != history_.end()) {
      ResampledTrack track;
      track.detection_id = track_id;
      track.samples.reserve(desired_times.size());
      for (const double desired_time : desired_times) {
        track.samples.push_back(nearestSampleForSlot(history_it->second, desired_time));
      }
      tracks.push_back(std::move(track));
    }
  }

  return tracks;
}

std::vector<SceneBuilder::TargetCandidate> SceneBuilder::selectTargets(
  const vision_msgs::msg::Detection3DArray & detections, const double current_time,
  const geometry_msgs::msg::PoseStamped & ego_pose) const
{
  std::vector<TargetCandidate> candidates;
  candidates.reserve(detections.detections.size());

  const auto & ego_position = ego_pose.pose.position;
  if (!isFinite(ego_position.x) || !isFinite(ego_position.y)) {
    return candidates;
  }

  const auto & ego_orientation = ego_pose.pose.orientation;
  if (!isFiniteQuaternion(ego_orientation)) {
    return candidates;
  }

  const double ego_yaw = yawFromQuaternion(ego_orientation);
  const double cos_yaw = std::cos(ego_yaw);
  const double sin_yaw = std::sin(ego_yaw);
  const double half_angle_rad =
    clampForwardHalfAngleDeg(config_.target_forward_half_angle_deg) * kDegToRad;

  for (const auto & detection : detections.detections) {
    const auto sample = sampleFromDetection(detection, current_time, detections.header.frame_id);
    if (!sample.has_value()) {
      continue;
    }

    if (history_.find(sample->detection_id) == history_.end()) {
      continue;
    }

    const double dx = sample->x - ego_position.x;
    const double dy = sample->y - ego_position.y;
    // Ego-frame: +lon along heading, +lat to the left.
    const double lon = dx * cos_yaw + dy * sin_yaw;
    const double lat = -dx * sin_yaw + dy * cos_yaw;
    const double bearing_abs = std::abs(std::atan2(lat, lon));

    TargetCandidate candidate;
    candidate.sample = *sample;
    candidate.distance_sq = dx * dx + dy * dy;
    candidate.in_forward_region = bearing_abs <= half_angle_rad;
    candidates.push_back(std::move(candidate));
  }

  std::sort(candidates.begin(), candidates.end(), [](const TargetCandidate & lhs, const TargetCandidate & rhs) {
    if (lhs.in_forward_region != rhs.in_forward_region) {
      return lhs.in_forward_region;
    }
    return lhs.distance_sq < rhs.distance_sq;
  });

  if (config_.selected_target_agent_limit < 0) {
    candidates.clear();
    return candidates;
  }

  const auto limit = static_cast<std::size_t>(config_.selected_target_agent_limit);
  if (candidates.size() > limit) {
    candidates.resize(limit);
  }

  return candidates;
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

int SceneBuilder::findTrackIndex(const std::vector<std::string> & track_ids, const std::string & detection_id)
{
  const auto it = std::find(track_ids.begin(), track_ids.end(), detection_id);
  if (it == track_ids.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(track_ids.begin(), it));
}

void SceneBuilder::fillTargetContextOutputs(
  const std::vector<TargetCandidate> & targets, const std::vector<std::string> & context_track_ids,
  const std::vector<ResampledTrack> & resampled_tracks, MtrInputTensors & tensors) const
{
  const std::size_t num_targets = targets.size();
  const std::size_t num_context = context_track_ids.size();
  const std::size_t num_steps = static_cast<std::size_t>(config_.history_steps);

  tensors.obj_trajs_mask.assign(num_targets * num_context * num_steps, 0U);
  tensors.obj_trajs_last_pos.assign(num_targets * num_context * 3U, 0.0F);

  for (std::size_t target_idx = 0; target_idx < num_targets; ++target_idx) {
    const auto & target = targets[target_idx];
    const int track_index = findTrackIndex(context_track_ids, target.sample.detection_id);
    if (track_index < 0) {
      continue;
    }

    MtrTargetSidecar sidecar;
    sidecar.detection_id = target.sample.detection_id;
    sidecar.track_index = track_index;
    sidecar.center_x = target.sample.x;
    sidecar.center_y = target.sample.y;
    sidecar.center_heading = target.sample.heading;
    tensors.sidecar.targets.push_back(sidecar);
    tensors.track_index_to_predict.push_back(static_cast<int32_t>(track_index));

    for (std::size_t context_idx = 0; context_idx < resampled_tracks.size(); ++context_idx) {
      const auto & track = resampled_tracks[context_idx];
      for (std::size_t step_idx = 0; step_idx < track.samples.size(); ++step_idx) {
        const std::size_t mask_idx = target_idx * num_context * num_steps + context_idx * num_steps + step_idx;
        tensors.obj_trajs_mask[mask_idx] = track.samples[step_idx].has_value() ? 1U : 0U;
      }

      for (auto sample_it = track.samples.rbegin(); sample_it != track.samples.rend(); ++sample_it) {
        if (!sample_it->has_value()) {
          continue;
        }

        const auto & sample = **sample_it;
        const std::size_t pos_idx = target_idx * num_context * 3U + context_idx * 3U;
        tensors.obj_trajs_last_pos[pos_idx] = static_cast<float>(sample.x);
        tensors.obj_trajs_last_pos[pos_idx + 1U] = static_cast<float>(sample.y);
        tensors.obj_trajs_last_pos[pos_idx + 2U] = static_cast<float>(sample.z);
        break;
      }
    }
  }
}

}  // namespace prediction_ml
