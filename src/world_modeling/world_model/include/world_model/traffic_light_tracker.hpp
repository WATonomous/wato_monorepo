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

#ifndef WORLD_MODEL__TRAFFIC_LIGHT_TRACKER_HPP_
#define WORLD_MODEL__TRAFFIC_LIGHT_TRACKER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "world_model/lanelet_handler.hpp"

namespace world_model
{

/**
 * Traffic light state constants.
 */
struct TrafficLightState
{
  static constexpr uint8_t UNKNOWN = 0;
  static constexpr uint8_t RED = 1;
  static constexpr uint8_t YELLOW = 2;
  static constexpr uint8_t GREEN = 3;
};

/**
 * Tracked traffic light with state and timing.
 */
struct TrackedTrafficLight
{
  int64_t reg_elem_id;           // Lanelet2 regulatory element ID
  uint8_t current_state;         // TrafficLightState value
  rclcpp::Time last_update;      // Timestamp of most recent detection
  float confidence;              // Detection confidence

  TrackedTrafficLight()
  : reg_elem_id(-1), current_state(TrafficLightState::UNKNOWN), confidence(0.0f) {}
};

/**
 * Maps 2D traffic light detections to lanelet regulatory elements
 * and tracks their states.
 */
class TrafficLightTracker
{
public:
  /**
   * Constructor.
   *
   * @param lanelet_handler Pointer to lanelet handler for regulatory element lookups
   * @param timeout_sec Time after which traffic light states become stale
   */
  TrafficLightTracker(
    LaneletHandler * lanelet_handler,
    double timeout_sec);

  ~TrafficLightTracker() = default;

  /**
   * Update traffic light states from 2D detections.
   *
   * @param detections Incoming 2D detection array (from traffic light detector)
   * @param current_time Current timestamp
   */
  void update(
    const vision_msgs::msg::Detection2DArray & detections,
    const rclcpp::Time & current_time);

  /**
   * Get the current state of a traffic light by regulatory element ID.
   *
   * @param reg_elem_id Lanelet2 regulatory element ID
   * @return Traffic light state if tracked, nullopt otherwise
   */
  std::optional<TrackedTrafficLight> getState(int64_t reg_elem_id) const;

  /**
   * Get the state of the traffic light controlling a specific lanelet.
   *
   * @param lanelet_id Lanelet ID
   * @return Traffic light state if the lanelet has a traffic light, nullopt otherwise
   */
  std::optional<TrackedTrafficLight> getStateForLanelet(int64_t lanelet_id) const;

  /**
   * Get all tracked traffic light states.
   */
  std::unordered_map<int64_t, TrackedTrafficLight> getAllStates() const;

  /**
   * Remove stale traffic light states.
   *
   * @param current_time Current timestamp
   */
  void pruneStale(const rclcpp::Time & current_time);

private:
  LaneletHandler * lanelet_handler_;
  std::unordered_map<int64_t, TrackedTrafficLight> traffic_lights_;
  double timeout_sec_;

  /**
   * Parse traffic light state from detection class label.
   */
  uint8_t parseState(const std::string & class_label) const;

  /**
   * Extract regulatory element ID from detection.
   * May use detection ID directly or perform spatial matching.
   */
  int64_t extractRegElemId(const vision_msgs::msg::Detection2D & detection) const;
};

}  // namespace world_model

#endif  // WORLD_MODEL__TRAFFIC_LIGHT_TRACKER_HPP_
