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

#include "prediction_ml/mtr_result_cache.hpp"

#include <cmath>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace prediction_ml
{
namespace
{
double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool finitePose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w);
}
}  // namespace

MtrResultCache::MtrResultCache(const double ttl_s)
: ttl_s_(ttl_s)
{}

void MtrResultCache::rememberRequest(const deep_msgs::msg::MtrScene & scene)
{
  Pending pending;
  pending.source_s = seconds(scene.header.stamp);
  pending.frame_id = scene.header.frame_id;
  for (const auto & detection : scene.detections.detections) {
    if (!detection.id.empty()) {
      pending.track_ids.insert(detection.id);
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = pending_.begin(); it != pending_.end();) {
    if (pending.source_s - it->second.source_s > ttl_s_) {
      it = pending_.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = cached_.begin(); it != cached_.end();) {
    if (pending.source_s - it->second.source_s > ttl_s_) {
      it = cached_.erase(it);
    } else {
      ++it;
    }
  }
  pending_[scene.request_id] = std::move(pending);
}

bool MtrResultCache::accept(const deep_msgs::msg::MtrPredictionArray & result, const double now_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto pending_it = pending_.find(result.request_id);
  if (pending_it == pending_.end()) {
    return false;
  }

  const Pending & pending = pending_it->second;
  const double result_source_s = seconds(result.header.stamp);
  const double age_s = now_s - pending.source_s;
  if (
    result.header.frame_id != pending.frame_id || result_source_s != pending.source_s || age_s < 0.0 ||
    age_s > ttl_s_ || result.objects.empty())
  {
    return false;
  }

  std::unordered_map<std::string, Cached> accepted;
  for (const auto & object : result.objects) {
    if (object.track_id.empty() || pending.track_ids.count(object.track_id) != 1 || object.trajectories.empty()) {
      return false;
    }

    Cached cached;
    cached.source_s = pending.source_s;
    for (const auto & trajectory : object.trajectories) {
      if (!std::isfinite(trajectory.confidence) || trajectory.confidence < 0.0F || trajectory.poses.empty()) {
        return false;
      }
      for (const auto & pose : trajectory.poses) {
        if (pose.header.frame_id != pending.frame_id || !finitePose(pose)) {
          return false;
        }
      }

      world_model_msgs::msg::Prediction prediction;
      prediction.header = result.header;
      prediction.conf = trajectory.confidence;
      prediction.poses = trajectory.poses;
      cached.predictions.push_back(std::move(prediction));
    }
    accepted[object.track_id] = std::move(cached);
  }

  for (auto & entry : accepted) {
    cached_[entry.first] = std::move(entry.second);
  }
  pending_.erase(pending_it);
  return true;
}

std::vector<world_model_msgs::msg::WorldObject> MtrResultCache::select(
  const std::vector<world_model_msgs::msg::WorldObject> & fallback, const double now_s) const
{
  auto output = fallback;
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & object : output) {
    const auto cached_it = cached_.find(object.detection.id);
    if (cached_it == cached_.end()) {
      continue;
    }
    const double age_s = now_s - cached_it->second.source_s;
    if (age_s >= 0.0 && age_s <= ttl_s_) {
      object.predictions = cached_it->second.predictions;
    }
  }
  return output;
}
}  // namespace prediction_ml
