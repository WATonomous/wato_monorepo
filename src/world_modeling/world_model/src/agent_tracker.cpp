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

#include "world_model/agent_tracker.hpp"

#include <algorithm>

namespace world_model
{

AgentTracker::AgentTracker(
  LaneletHandler * lanelet_handler,
  double history_duration_sec,
  double prune_timeout_sec)
: lanelet_handler_(lanelet_handler),
  history_duration_sec_(history_duration_sec),
  prune_timeout_sec_(prune_timeout_sec)
{
}

void AgentTracker::updateDetections(
  const vision_msgs::msg::Detection3DArray & detections,
  const rclcpp::Time & current_time)
{
  for (const auto & detection : detections.detections) {
    int64_t track_id = extractTrackId(detection);
    if (track_id < 0) {
      continue;  // Skip detections without valid track ID
    }

    // Get or create agent entry
    auto & agent = agents_[track_id];
    if (agent.track_id < 0) {
      agent.track_id = track_id;
    }

    // Update agent data
    agent.last_seen = current_time;
    agent.object_class = extractObjectClass(detection);
    agent.history.push_back(detection);

    // Prune old history entries
    pruneAgentHistory(agent, current_time);

    // Update lanelet context
    if (lanelet_handler_ && lanelet_handler_->isMapLoaded()) {
      geometry_msgs::msg::Point position;
      position.x = detection.bbox.center.position.x;
      position.y = detection.bbox.center.position.y;
      position.z = detection.bbox.center.position.z;

      auto nearest_ll = lanelet_handler_->findNearestLanelet(position);
      if (nearest_ll.has_value()) {
        agent.current_lanelet_id = nearest_ll->id();
      } else {
        agent.current_lanelet_id = -1;
      }
    }
  }

  // Prune agents that haven't been seen recently
  pruneOldAgents(current_time);
}

void AgentTracker::updatePredictions(
  const prediction_msgs::msg::PredictionHypothesesArray & predictions)
{
  for (const auto & pred_hypotheses : predictions.pred_h_arr) {
    int64_t agent_id = pred_hypotheses.id;

    // Find the agent with this ID
    auto it = agents_.find(agent_id);
    if (it == agents_.end()) {
      // Agent not currently tracked, skip
      continue;
    }

    // Update the agent's predictions
    it->second.predictions.clear();
    it->second.predictions.reserve(pred_hypotheses.preds.size());
    for (const auto & pred : pred_hypotheses.preds) {
      it->second.predictions.push_back(pred);
    }
  }
}

void AgentTracker::pruneOldAgents(const rclcpp::Time & current_time)
{
  auto it = agents_.begin();
  while (it != agents_.end()) {
    double age = (current_time - it->second.last_seen).seconds();
    if (age > prune_timeout_sec_) {
      it = agents_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<TrackedAgent> AgentTracker::getTrackedAgents() const
{
  std::vector<TrackedAgent> result;
  result.reserve(agents_.size());
  for (const auto & pair : agents_) {
    result.push_back(pair.second);
  }
  return result;
}

std::optional<TrackedAgent> AgentTracker::getAgent(int64_t track_id) const
{
  auto it = agents_.find(track_id);
  if (it != agents_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<TrackedAgent> AgentTracker::getAgentsOnLanelet(int64_t lanelet_id) const
{
  std::vector<TrackedAgent> result;
  for (const auto & pair : agents_) {
    if (pair.second.current_lanelet_id == lanelet_id) {
      result.push_back(pair.second);
    }
  }
  return result;
}

size_t AgentTracker::getAgentCount() const
{
  return agents_.size();
}

int64_t AgentTracker::extractTrackId(const vision_msgs::msg::Detection3D & detection)
{
  // Try to get tracking_id from the detection
  // The tracking_id field was added in vision_msgs
  // If not available, use the id from the first result

  // First check if there's a tracking_id in the detection
  // Note: vision_msgs::msg::Detection3D has a tracking_id field
  if (!detection.tracking_id.empty()) {
    // tracking_id is a string, try to parse as integer
    try {
      return std::stoll(detection.tracking_id);
    } catch (const std::exception &) {
      // Fall through to use results
    }
  }

  // Fallback: use the ID from the first hypothesis result
  if (!detection.results.empty() && !detection.results[0].hypothesis.class_id.empty()) {
    // Try to parse class_id as track id (some trackers put it there)
    try {
      return std::stoll(detection.results[0].hypothesis.class_id);
    } catch (const std::exception &) {
      // Not a valid integer
    }
  }

  // If we still don't have an ID, return -1 to indicate invalid
  return -1;
}

std::string AgentTracker::extractObjectClass(const vision_msgs::msg::Detection3D & detection)
{
  if (!detection.results.empty()) {
    // Return the class with highest score
    float max_score = -1.0f;
    std::string best_class;

    for (const auto & result : detection.results) {
      if (result.hypothesis.score > max_score) {
        max_score = result.hypothesis.score;
        best_class = result.hypothesis.class_id;
      }
    }
    return best_class;
  }
  return "unknown";
}

void AgentTracker::pruneAgentHistory(TrackedAgent & agent, const rclcpp::Time & current_time)
{
  while (!agent.history.empty()) {
    rclcpp::Time oldest_time(agent.history.front().header.stamp);
    double age = (current_time - oldest_time).seconds();
    if (age > history_duration_sec_) {
      agent.history.pop_front();
    } else {
      break;
    }
  }
}

}  // namespace world_model
