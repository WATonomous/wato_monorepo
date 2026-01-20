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

#include "world_model/traffic_light_tracker.hpp"

#include <algorithm>

namespace world_model
{

TrafficLightTracker::TrafficLightTracker(
  LaneletHandler * lanelet_handler,
  double timeout_sec)
: lanelet_handler_(lanelet_handler),
  timeout_sec_(timeout_sec)
{
}

void TrafficLightTracker::update(
  const vision_msgs::msg::Detection2DArray & detections,
  const rclcpp::Time & current_time)
{
  for (const auto & detection : detections.detections) {
    int64_t reg_elem_id = extractRegElemId(detection);
    if (reg_elem_id < 0) {
      continue;  // Skip if we can't determine the regulatory element
    }

    auto & tl = traffic_lights_[reg_elem_id];
    tl.reg_elem_id = reg_elem_id;
    tl.last_update = current_time;

    // Get state and confidence from detection results
    if (!detection.results.empty()) {
      float max_score = -1.0f;
      std::string best_class;

      for (const auto & result : detection.results) {
        if (result.hypothesis.score > max_score) {
          max_score = result.hypothesis.score;
          best_class = result.hypothesis.class_id;
        }
      }

      tl.current_state = parseState(best_class);
      tl.confidence = max_score;
    }
  }

  // Prune stale entries
  pruneStale(current_time);
}

std::optional<TrackedTrafficLight> TrafficLightTracker::getState(int64_t reg_elem_id) const
{
  auto it = traffic_lights_.find(reg_elem_id);
  if (it != traffic_lights_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::optional<TrackedTrafficLight> TrafficLightTracker::getStateForLanelet(int64_t lanelet_id) const
{
  if (!lanelet_handler_ || !lanelet_handler_->isMapLoaded()) {
    return std::nullopt;
  }

  auto ll_opt = lanelet_handler_->getLaneletById(lanelet_id);
  if (!ll_opt.has_value()) {
    return std::nullopt;
  }

  // Check if any of the lanelet's regulatory elements is a tracked traffic light
  for (const auto & re : ll_opt->regulatoryElements()) {
    auto tl_state = getState(re->id());
    if (tl_state.has_value()) {
      return tl_state;
    }
  }

  return std::nullopt;
}

std::unordered_map<int64_t, TrackedTrafficLight> TrafficLightTracker::getAllStates() const
{
  return traffic_lights_;
}

void TrafficLightTracker::pruneStale(const rclcpp::Time & current_time)
{
  auto it = traffic_lights_.begin();
  while (it != traffic_lights_.end()) {
    double age = (current_time - it->second.last_update).seconds();
    if (age > timeout_sec_) {
      it = traffic_lights_.erase(it);
    } else {
      ++it;
    }
  }
}

uint8_t TrafficLightTracker::parseState(const std::string & class_label) const
{
  // Convert to lowercase for comparison
  std::string lower = class_label;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower.find("red") != std::string::npos) {
    return TrafficLightState::RED;
  } else if (lower.find("yellow") != std::string::npos ||
    lower.find("amber") != std::string::npos)
  {
    return TrafficLightState::YELLOW;
  } else if (lower.find("green") != std::string::npos) {
    return TrafficLightState::GREEN;
  }

  return TrafficLightState::UNKNOWN;
}

int64_t TrafficLightTracker::extractRegElemId(const vision_msgs::msg::Detection2D & detection) const
{
  // The detection ID might directly map to a regulatory element ID
  // This depends on how the traffic light detector is configured

  // Try to parse the id field as the regulatory element ID
  if (!detection.id.empty()) {
    try {
      return std::stoll(detection.id);
    } catch (const std::exception &) {
      // Not a valid integer
    }
  }

  // Alternative: use tracking_id if available
  if (!detection.tracking_id.empty()) {
    try {
      return std::stoll(detection.tracking_id);
    } catch (const std::exception &) {
      // Not a valid integer
    }
  }

  // If we can't determine the regulatory element ID, return -1
  return -1;
}

}  // namespace world_model
