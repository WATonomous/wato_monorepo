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
#include <unordered_set>
#include <utility>
#include <vector>

namespace prediction_ml
{
namespace
{

constexpr double kUnsetTimestamp = 0.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// Reserved history key for the ego / self-driving-car track (is_sdc row). 
constexpr char kEgoTrackId[] = "__ego__";

// Fixed TRT engine contract capacities (leading dims of the input tensors).
constexpr std::size_t kTargetCapacity = 8;    // obj_trajs dim 0
constexpr std::size_t kContextCapacity = 128;  // obj_trajs dim 1

// Frozen map-tensor shape [8, 768, 20, 9]; the map buffers are not packed yet but the shape is
// published now so binding validation is stable.
constexpr int64_t kNumSrcPolylines = 768;
constexpr int64_t kNumPointsPerPolyline = 20;
constexpr int64_t kMapFeatureDim = 9;

// INT64 type codes the engine consumes and the matching debug tokens (correct spelling).
constexpr int64_t kTypeVehicle = 0;
constexpr int64_t kTypePedestrian = 1;
constexpr int64_t kTypeCyclist = 2;

// obj_trajs generated feature layout, dim F = history_steps + 18:
//   [0..2]   relative x, y, z            (target-centered)
//   [3..5]   dx, dy, dz                  (box dims, untransformed)
//   [6..10]  one-hot: veh, ped, cyc, is_center_object, is_sdc
//   [11..11+T-1] time one-hot (this step = 1)
//   [11+T]   scalar timestamp            (slot time relative to current frame, seconds)
//   [12+T..13+T] sin, cos of relative heading
//   [14+T..15+T] relative vx, vy
//   [16+T..17+T] ax, ay                  (target-centered acceleration)
constexpr std::size_t kBoxOffset = 0;
constexpr std::size_t kOneHotOffset = 6;
constexpr std::size_t kTimeOffset = 11;  // time one-hot starts here; scalar at kTimeOffset + T

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

double wrapToPi(double angle)
{
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
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

  if (!frame.has_ego) {
    return tensors;
  }

  // Ego joins history as the SDC track before pruning/context so it can be resampled
  addEgoSample(frame.ego_pose, *current_time);
  pruneHistory(*current_time);

  const auto targets = selectTargets(frame.detections, *current_time, frame.ego_pose);
  if (targets.empty()) {
    return tensors;
  }

  const auto context_track_ids = contextTrackIds(frame.ego_pose);
  if (context_track_ids.empty()) {
    return tensors;
  }

  const auto desired_times = desiredSampleTimes(*current_time);
  const auto resampled_tracks = resampleTracks(context_track_ids, desired_times);

  // packObjectTensors returns false (hard frame failure) if a selected target is missing from the
  // 128-row context; in that case tensors stay untouched and invalid.
  if (!packObjectTensors(targets, context_track_ids, resampled_tracks, desired_times, *current_time, tensors)) {
    return tensors;
  }

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

void SceneBuilder::addEgoSample(const geometry_msgs::msg::PoseStamped & ego_pose, const double timestamp)
{
  if (!config_valid_ || !isUsableTimestamp(timestamp)) {
    return;
  }

  const auto & position = ego_pose.pose.position;
  const auto & orientation = ego_pose.pose.orientation;
  if (
    !isFinite(position.x) || !isFinite(position.y) || !isFinite(position.z) ||
    !isFiniteQuaternion(orientation)) {
    return;
  }

  const double heading = yawFromQuaternion(orientation);
  if (!isFinite(heading)) {
    return;
  }

  HistorySample sample;
  sample.detection_id = kEgoTrackId;
  sample.timestamp = timestamp;
  sample.x = position.x;
  sample.y = position.y;
  sample.z = position.z;
  // Ego bbox dims are unknown from a bare pose; leave zero until an ego-footprint config exists.
  sample.heading = heading;
  sample.object_class = MtrObjectClass::Vehicle;
  sample.frame_id = ego_pose.header.frame_id;

  // Finite-difference ego velocity from the previous ego sample (odom is the future upgrade path).
  const auto it = history_.find(kEgoTrackId);
  if (it != history_.end() && !it->second.empty()) {
    const auto & prev = it->second.back();
    const double dt = timestamp - prev.timestamp;
    if (isFinite(dt) && dt > 0.0) {
      sample.vx = (position.x - prev.x) / dt;
      sample.vy = (position.y - prev.y) / dt;
      sample.vz = (position.z - prev.z) / dt;
    }
  }

  addSample(sample);
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
    ResampledTrack track;
    track.detection_id = track_id;
    track.samples.reserve(desired_times.size());

    const auto history_it = history_.find(track_id);
    if (history_it != history_.end()) {
      for (const double desired_time : desired_times) {
        track.samples.push_back(nearestSampleForSlot(history_it->second, desired_time));
      }
    } else {
      track.samples.assign(desired_times.size(), std::nullopt);
    }
    tracks.push_back(std::move(track));
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

  // One detection_id cannot occupy two target slots.
  std::unordered_set<std::string> accepted_ids;

  for (const auto & detection : detections.detections) {
    const auto sample = sampleFromDetection(detection, current_time, detections.header.frame_id);
    if (!sample.has_value()) {
      continue;
    }

    if (history_.find(sample->detection_id) == history_.end()) {
      continue;
    }

    if (!accepted_ids.insert(sample->detection_id).second) {
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

std::vector<std::string> SceneBuilder::contextTrackIds(const geometry_msgs::msg::PoseStamped & ego_pose) const
{
  const auto & ego_position = ego_pose.pose.position;

  std::vector<ContextTrack> tracks;
  tracks.reserve(history_.size());
  for (const auto & entry : history_) {
    if (entry.second.empty()) {
      continue;
    }
    const auto & latest = entry.second.back();
    const double dx = latest.x - ego_position.x;
    const double dy = latest.y - ego_position.y;
    tracks.push_back(ContextTrack{entry.first, dx * dx + dy * dy});
  }

  // Closest first by ego-frame 2D distance; tie-break detection_id ascending (strict weak order).
  // Ego sits at distance 0 and therefore always lands in row 0.
  std::sort(tracks.begin(), tracks.end(), [](const ContextTrack & lhs, const ContextTrack & rhs) {
    if (lhs.distance_sq != rhs.distance_sq) {
      return lhs.distance_sq < rhs.distance_sq;
    }
    return lhs.detection_id < rhs.detection_id;
  });

  if (tracks.size() > kContextCapacity) {
    tracks.resize(kContextCapacity);
  }

  std::vector<std::string> ids;
  ids.reserve(tracks.size());
  for (const auto & track : tracks) {
    ids.push_back(track.detection_id);
  }
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

bool SceneBuilder::packObjectTensors(
  const std::vector<TargetCandidate> & targets, const std::vector<std::string> & context_track_ids,
  const std::vector<ResampledTrack> & resampled_tracks, const std::vector<double> & desired_times,
  const double current_time, MtrInputTensors & tensors) const
{
  const std::size_t num_steps = static_cast<std::size_t>(config_.history_steps);
  const std::size_t feature_dim = num_steps + 18;  // generated MTR layout: 6+5+(T+1)+2+2+2
  const std::size_t num_real_targets = std::min(targets.size(), kTargetCapacity);

  // Resolve every selected target's context row up front; a missing target is a hard frame
  // failure so we bail before writing anything.
  std::vector<std::size_t> target_track_index(num_real_targets, 0);
  for (std::size_t t = 0; t < num_real_targets; ++t) {
    const int idx = findTrackIndex(context_track_ids, targets[t].sample.detection_id);
    if (idx < 0 || static_cast<std::size_t>(idx) >= kContextCapacity) {
      return false;
    }
    target_track_index[t] = static_cast<std::size_t>(idx);
  }

  // Fixed-capacity, zero-initialized buffers. Unused target/context/step slots stay zero with
  // mask = false; unused target metadata slots hold the 0 sentinels (NOT -1).
  tensors.obj_trajs.assign(kTargetCapacity * kContextCapacity * num_steps * feature_dim, 0.0F);
  tensors.obj_trajs_mask.assign(kTargetCapacity * kContextCapacity * num_steps, 0U);
  tensors.obj_trajs_last_pos.assign(kTargetCapacity * kContextCapacity * 3U, 0.0F);
  tensors.track_index_to_predict.assign(kTargetCapacity, 0);
  tensors.center_type_ids.assign(kTargetCapacity, 0);
  tensors.center_objects_type.assign(kTargetCapacity, std::string{});
  tensors.obj_trajs_shape = {
    static_cast<int64_t>(kTargetCapacity), static_cast<int64_t>(kContextCapacity),
    static_cast<int64_t>(num_steps), static_cast<int64_t>(feature_dim)};
  tensors.map_polylines_shape = {
    static_cast<int64_t>(kTargetCapacity), kNumSrcPolylines, kNumPointsPerPolyline, kMapFeatureDim};

  const std::size_t num_context = std::min(resampled_tracks.size(), kContextCapacity);
  const std::size_t scalar_time_idx = kTimeOffset + num_steps;  // time one-hot [11..11+T-1], scalar at 11+T

  for (std::size_t t = 0; t < num_real_targets; ++t) {
    const auto & center = targets[t].sample;
    const double cos_h = std::cos(center.heading);
    const double sin_h = std::sin(center.heading);

    int64_t type_code = kTypeVehicle;
    std::string type_token = "TYPE_VEHICLE";
    switch (center.object_class) {
      case MtrObjectClass::Vehicle:
        type_code = kTypeVehicle;
        type_token = "TYPE_VEHICLE";
        break;
      case MtrObjectClass::Pedestrian:
        type_code = kTypePedestrian;
        type_token = "TYPE_PEDESTRIAN";
        break;
      case MtrObjectClass::Cyclist:
        type_code = kTypeCyclist;
        type_token = "TYPE_CYCLIST";
        break;
    }

    tensors.track_index_to_predict[t] = static_cast<int64_t>(target_track_index[t]);
    tensors.center_type_ids[t] = type_code;
    tensors.center_objects_type[t] = type_token;

    MtrTargetSidecar sidecar;
    sidecar.detection_id = center.detection_id;
    sidecar.track_index = static_cast<int>(target_track_index[t]);
    sidecar.center_x = center.x;   // scene frame, so Person C can rotate predictions back
    sidecar.center_y = center.y;
    sidecar.center_heading = center.heading;
    tensors.sidecar.targets.push_back(sidecar);

    for (std::size_t c = 0; c < num_context; ++c) {
      const auto & track = resampled_tracks[c];
      const bool is_sdc = (track.detection_id == kEgoTrackId);
      const bool is_center = (track.detection_id == center.detection_id);

      // Pass 1: target-centered velocity + presence per step (needed before acceleration).
      std::vector<double> rel_vx(num_steps, 0.0);
      std::vector<double> rel_vy(num_steps, 0.0);
      std::vector<char> present(num_steps, 0);
      for (std::size_t s = 0; s < num_steps && s < track.samples.size(); ++s) {
        if (!track.samples[s].has_value()) {
          continue;
        }
        const auto & smp = *track.samples[s];
        present[s] = 1;
        rel_vx[s] = smp.vx * cos_h + smp.vy * sin_h;
        rel_vy[s] = -smp.vx * sin_h + smp.vy * cos_h;
      }

      // Pass 2: acceleration a_i = (v_i - v_{i-1}) * rate when both slots present; the first valid
      // slot copies the next valid acceleration, else zero.
      std::vector<double> acc_x(num_steps, 0.0);
      std::vector<double> acc_y(num_steps, 0.0);
      std::vector<char> acc_valid(num_steps, 0);
      for (std::size_t s = 1; s < num_steps; ++s) {
        if (present[s] && present[s - 1]) {
          acc_x[s] = (rel_vx[s] - rel_vx[s - 1]) * config_.history_rate_hz;
          acc_y[s] = (rel_vy[s] - rel_vy[s - 1]) * config_.history_rate_hz;
          acc_valid[s] = 1;
        }
      }
      for (std::size_t s = 0; s < num_steps; ++s) {
        if (!present[s]) {
          continue;
        }
        if (!acc_valid[s]) {
          for (std::size_t g = s + 1; g < num_steps; ++g) {
            if (acc_valid[g]) {
              acc_x[s] = acc_x[g];
              acc_y[s] = acc_y[g];
              break;
            }
          }
        }
        break;  // only the earliest present slot inherits
      }

      // Pass 3: write per-step features for present slots (masked slots stay zero).
      for (std::size_t s = 0; s < num_steps && s < track.samples.size(); ++s) {
        if (!present[s]) {
          continue;
        }
        const std::size_t mask_idx = (t * kContextCapacity + c) * num_steps + s;
        tensors.obj_trajs_mask[mask_idx] = 1U;

        const auto & smp = *track.samples[s];
        const double px = smp.x - center.x;
        const double py = smp.y - center.y;
        const double rx = px * cos_h + py * sin_h;
        const double ry = -px * sin_h + py * cos_h;
        const double rz = smp.z - center.z;
        const double rel_heading = wrapToPi(smp.heading - center.heading);

        const std::size_t base = ((t * kContextCapacity + c) * num_steps + s) * feature_dim;
        float * feat = &tensors.obj_trajs[base];
        feat[kBoxOffset + 0] = static_cast<float>(rx);
        feat[kBoxOffset + 1] = static_cast<float>(ry);
        feat[kBoxOffset + 2] = static_cast<float>(rz);
        feat[kBoxOffset + 3] = static_cast<float>(smp.dx);
        feat[kBoxOffset + 4] = static_cast<float>(smp.dy);
        feat[kBoxOffset + 5] = static_cast<float>(smp.dz);
        feat[kOneHotOffset + 0] = (smp.object_class == MtrObjectClass::Vehicle) ? 1.0F : 0.0F;
        feat[kOneHotOffset + 1] = (smp.object_class == MtrObjectClass::Pedestrian) ? 1.0F : 0.0F;
        feat[kOneHotOffset + 2] = (smp.object_class == MtrObjectClass::Cyclist) ? 1.0F : 0.0F;
        feat[kOneHotOffset + 3] = is_center ? 1.0F : 0.0F;
        feat[kOneHotOffset + 4] = is_sdc ? 1.0F : 0.0F;
        feat[kTimeOffset + s] = 1.0F;                                             // time one-hot
        feat[scalar_time_idx] = static_cast<float>(desired_times[s] - current_time);  // scalar time
        feat[scalar_time_idx + 1] = static_cast<float>(std::sin(rel_heading));
        feat[scalar_time_idx + 2] = static_cast<float>(std::cos(rel_heading));
        feat[scalar_time_idx + 3] = static_cast<float>(rel_vx[s]);
        feat[scalar_time_idx + 4] = static_cast<float>(rel_vy[s]);
        feat[scalar_time_idx + 5] = static_cast<float>(acc_x[s]);
        feat[scalar_time_idx + 6] = static_cast<float>(acc_y[s]);
      }

      // obj_trajs_last_pos: latest present slot, target-centered.
      for (std::size_t s = num_steps; s-- > 0;) {
        if (s >= track.samples.size() || !present[s]) {
          continue;
        }
        const auto & smp = *track.samples[s];
        const double px = smp.x - center.x;
        const double py = smp.y - center.y;
        const std::size_t pos_idx = (t * kContextCapacity + c) * 3U;
        tensors.obj_trajs_last_pos[pos_idx + 0] = static_cast<float>(px * cos_h + py * sin_h);
        tensors.obj_trajs_last_pos[pos_idx + 1] = static_cast<float>(-px * sin_h + py * cos_h);
        tensors.obj_trajs_last_pos[pos_idx + 2] = static_cast<float>(smp.z - center.z);
        break;
      }
    }
  }

  return true;
}

}  // namespace prediction_ml
