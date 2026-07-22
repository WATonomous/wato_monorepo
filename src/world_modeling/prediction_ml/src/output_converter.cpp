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

#include "prediction_ml/output_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace prediction_ml
{
namespace
{

constexpr double kDirectionEpsilonSquared = 1.0e-12;

bool checkedElementCount(const std::vector<int64_t> & shape, std::size_t & count)
{
  count = 1;
  for (const int64_t dimension : shape) {
    if (dimension <= 0) {
      return false;
    }
    const auto size = static_cast<std::size_t>(dimension);
    if (count > std::numeric_limits<std::size_t>::max() / size) {
      return false;
    }
    count *= size;
  }
  return true;
}

builtin_interfaces::msg::Time timeFromSeconds(const double seconds)
{
  builtin_interfaces::msg::Time stamp;
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return stamp;
  }

  const long double nanoseconds = static_cast<long double>(seconds) * 1.0e9L;
  if (nanoseconds > static_cast<long double>(std::numeric_limits<int64_t>::max())) {
    return stamp;
  }
  const int64_t rounded_nanoseconds = static_cast<int64_t>(std::llround(nanoseconds));
  const int64_t seconds_part = rounded_nanoseconds / 1000000000LL;
  if (seconds_part > std::numeric_limits<int32_t>::max()) {
    return stamp;
  }
  stamp.sec = static_cast<int32_t>(seconds_part);
  stamp.nanosec = static_cast<uint32_t>(rounded_nanoseconds % 1000000000LL);
  return stamp;
}

bool finiteTarget(const MtrTargetSidecar & target)
{
  return !target.detection_id.empty() && std::isfinite(target.center_x) && std::isfinite(target.center_y) &&
         std::isfinite(target.center_heading);
}

}  // namespace

MtrInferenceResult convertMtrOutput(
  const MtrOutputTensors & out,
  const MtrBatchSidecar & sidecar,
  const std::string & frame_id,
  const double source_time_s,
  const double horizon_s,
  const double time_step_s)
{
  MtrInferenceResult result;
  if (!out.valid) {
    result.error = "mtr output invalid";
    return result;
  }

  const std::string output_frame = frame_id.empty() ? sidecar.frame_id : frame_id;
  if (
    output_frame.empty() || !std::isfinite(source_time_s) || source_time_s < 0.0 || !std::isfinite(horizon_s) ||
    horizon_s <= 0.0 || !std::isfinite(time_step_s) || time_step_s <= 0.0)
  {
    result.error = "invalid output frame or timing configuration";
    return result;
  }
  if (out.scores_shape.size() != 2U || out.trajs_shape.size() != 4U) {
    result.error = "mtr output rank mismatch";
    return result;
  }

  std::size_t scores_count = 0;
  std::size_t trajs_count = 0;
  if (
    !checkedElementCount(out.scores_shape, scores_count) || !checkedElementCount(out.trajs_shape, trajs_count) ||
    scores_count != out.pred_scores.size() || trajs_count != out.pred_trajs.size())
  {
    result.error = "mtr output buffer size mismatch";
    return result;
  }

  const auto num_targets = static_cast<std::size_t>(out.scores_shape[0]);
  const auto num_modes = static_cast<std::size_t>(out.scores_shape[1]);
  const auto traj_targets = static_cast<std::size_t>(out.trajs_shape[0]);
  const auto traj_modes = static_cast<std::size_t>(out.trajs_shape[1]);
  const auto model_steps = static_cast<std::size_t>(out.trajs_shape[2]);
  const auto channels = static_cast<std::size_t>(out.trajs_shape[3]);
  if (
    num_targets != traj_targets || num_modes != traj_modes || channels < 2U || sidecar.targets.empty() ||
    sidecar.targets.size() > num_targets)
  {
    result.error = "mtr output shape does not match sidecar";
    return result;
  }

  const auto requested_steps =
    static_cast<std::size_t>(std::floor(horizon_s / time_step_s + std::numeric_limits<double>::epsilon() * 8.0));
  const std::size_t emitted_steps = std::min(model_steps, requested_steps);
  if (emitted_steps == 0U) {
    result.error = "mtr output contains no steps inside the requested horizon";
    return result;
  }

  std::unordered_set<std::string> emitted_ids;
  for (std::size_t target_index = 0; target_index < sidecar.targets.size(); ++target_index) {
    const auto & target = sidecar.targets[target_index];
    if (!finiteTarget(target) || !emitted_ids.insert(target.detection_id).second) {
      continue;
    }

    MtrObjectPrediction object;
    object.detection_id = target.detection_id;
    const double cos_heading = std::cos(target.center_heading);
    const double sin_heading = std::sin(target.center_heading);

    for (std::size_t mode_index = 0; mode_index < num_modes; ++mode_index) {
      const float score = out.pred_scores[target_index * num_modes + mode_index];
      if (!std::isfinite(score) || score <= 0.0F || score > 1.0F) {
        continue;
      }

      std::vector<geometry_msgs::msg::Point> positions;
      positions.reserve(emitted_steps);
      bool valid_mode = true;
      for (std::size_t step_index = 0; step_index < emitted_steps; ++step_index) {
        const std::size_t offset = (((target_index * num_modes + mode_index) * model_steps + step_index) * channels);
        const double local_x = out.pred_trajs[offset];
        const double local_y = out.pred_trajs[offset + 1U];
        if (!std::isfinite(local_x) || !std::isfinite(local_y)) {
          valid_mode = false;
          break;
        }

        geometry_msgs::msg::Point point;
        point.x = target.center_x + local_x * cos_heading - local_y * sin_heading;
        point.y = target.center_y + local_x * sin_heading + local_y * cos_heading;
        point.z = 0.0;
        positions.push_back(point);
      }
      if (!valid_mode || positions.empty()) {
        continue;
      }

      world_model_msgs::msg::Prediction prediction;
      prediction.header.frame_id = output_frame;
      prediction.header.stamp = timeFromSeconds(source_time_s);
      prediction.conf = static_cast<double>(score);
      prediction.poses.reserve(positions.size());

      double last_yaw = target.center_heading;
      for (std::size_t step_index = 0; step_index < positions.size(); ++step_index) {
        double dx = 0.0;
        double dy = 0.0;
        if (step_index + 1U < positions.size()) {
          dx = positions[step_index + 1U].x - positions[step_index].x;
          dy = positions[step_index + 1U].y - positions[step_index].y;
        } else if (step_index > 0U) {
          dx = positions[step_index].x - positions[step_index - 1U].x;
          dy = positions[step_index].y - positions[step_index - 1U].y;
        }
        if (dx * dx + dy * dy > kDirectionEpsilonSquared) {
          last_yaw = std::atan2(dy, dx);
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = output_frame;
        pose.header.stamp = timeFromSeconds(source_time_s + (step_index + 1U) * time_step_s);
        pose.pose.position = positions[step_index];
        pose.pose.orientation.z = std::sin(last_yaw * 0.5);
        pose.pose.orientation.w = std::cos(last_yaw * 0.5);
        prediction.poses.push_back(std::move(pose));
      }
      object.predictions.push_back(std::move(prediction));
    }

    if (!object.predictions.empty()) {
      std::stable_sort(object.predictions.begin(), object.predictions.end(), [](const auto & lhs, const auto & rhs) {
        return lhs.conf > rhs.conf;
      });
      result.objects.push_back(std::move(object));
    }
  }

  if (result.objects.empty()) {
    result.error = "mtr output had no valid per-object predictions";
    return result;
  }
  result.ok = true;
  return result;
}

}  // namespace prediction_ml
