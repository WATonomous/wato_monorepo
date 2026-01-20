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

#ifndef WORLD_MODEL__AGENT_TRACKER_HPP_
#define WORLD_MODEL__AGENT_TRACKER_HPP_

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "prediction_msgs/msg/prediction.hpp"
#include "prediction_msgs/msg/prediction_hypotheses_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model/lanelet_handler.hpp"

namespace world_model
{

/**
 * Tracked agent with detection history and lanelet context.
 */
struct TrackedAgent
{
  int64_t track_id;                                    // From detection.id or tracking_id
  std::deque<vision_msgs::msg::Detection3D> history;   // Past n seconds of detections
  rclcpp::Time last_seen;                              // Timestamp of most recent detection
  int64_t current_lanelet_id;                          // -1 if not on a lanelet
  std::string object_class;                            // Classification label

  // Predicted future paths (from Prediction node)
  std::vector<prediction_msgs::msg::Prediction> predictions;  // Multiple hypotheses with confidence

  TrackedAgent()
  : track_id(-1), current_lanelet_id(-1) {}
};

/**
 * Buffers 3D detections by track ID and enriches with lanelet context.
 *
 * Note: No temporal association/filtering - detections already have track IDs
 * from upstream tracker. This class simply stores history and provides lanelet
 * context enrichment.
 */
class AgentTracker
{
public:
  /**
   * Constructor.
   *
   * @param lanelet_handler Pointer to lanelet handler for lanelet lookups
   * @param history_duration_sec Duration to keep detection history
   * @param prune_timeout_sec Time after which unseen agents are removed
   */
  AgentTracker(
    LaneletHandler * lanelet_handler,
    double history_duration_sec,
    double prune_timeout_sec);

  ~AgentTracker() = default;

  /**
   * Update with new detections. Stores by existing track ID and enriches
   * with lanelet context.
   *
   * @param detections Incoming 3D detection array
   * @param current_time Current timestamp for pruning
   */
  void updateDetections(
    const vision_msgs::msg::Detection3DArray & detections,
    const rclcpp::Time & current_time);

  /**
   * Update predictions from the Prediction node.
   * Matches predictions to tracked agents by ID.
   *
   * @param predictions Incoming prediction hypotheses array
   */
  void updatePredictions(const prediction_msgs::msg::PredictionHypothesesArray & predictions);

  /**
   * Remove agents that haven't been seen recently.
   *
   * @param current_time Current timestamp
   */
  void pruneOldAgents(const rclcpp::Time & current_time);

  /**
   * Get all currently tracked agents.
   */
  std::vector<TrackedAgent> getTrackedAgents() const;

  /**
   * Get a specific agent by track ID.
   */
  std::optional<TrackedAgent> getAgent(int64_t track_id) const;

  /**
   * Get agents currently on a specific lanelet.
   */
  std::vector<TrackedAgent> getAgentsOnLanelet(int64_t lanelet_id) const;

  /**
   * Get the number of tracked agents.
   */
  size_t getAgentCount() const;

private:
  LaneletHandler * lanelet_handler_;
  std::unordered_map<int64_t, TrackedAgent> agents_;
  double history_duration_sec_;
  double prune_timeout_sec_;

  /**
   * Extract track ID from detection.
   * Uses tracking_id field if available, otherwise uses results[0].id.
   */
  int64_t extractTrackId(const vision_msgs::msg::Detection3D & detection);

  /**
   * Get object class label from detection results.
   */
  std::string extractObjectClass(const vision_msgs::msg::Detection3D & detection);

  /**
   * Prune old entries from a single agent's history.
   */
  void pruneAgentHistory(TrackedAgent & agent, const rclcpp::Time & current_time);
};

}  // namespace world_model

#endif  // WORLD_MODEL__AGENT_TRACKER_HPP_
