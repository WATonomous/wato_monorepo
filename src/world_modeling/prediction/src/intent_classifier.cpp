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

#include "prediction/intent_classifier.hpp"

#include <algorithm>  // for std::max, std::min
#include <cmath>
#include <numeric>
#include <vector>  // for std::vector

namespace prediction
{

IntentClassifier::IntentClassifier(rclcpp_lifecycle::LifecycleNode * node)
: node_(node)
{
  weights_.velocity_weight = 0.3;
  weights_.heading_weight = 0.2;
  weights_.intersection_weight = 0.25;
  weights_.lateral_offset_weight = 0.15;
  weights_.turn_signal_weight = 0.1;

  RCLCPP_INFO(node_->get_logger(), "IntentClassifier initialized");
}

void IntentClassifier::assignProbabilities(
  const vision_msgs::msg::Detection3D & detection,
  std::vector<TrajectoryHypothesis> & hypotheses,
  const IntentFeatures & features)
{
  if (hypotheses.empty()) {
    return;
  }

  // Compute probability for each hypothesis
  for (auto & hypothesis : hypotheses) {
    hypothesis.probability = computeIntentProbability(hypothesis.intent, features);
  }

  // Normalize probabilities to sum to 1.0
  normalizeProbabilities(hypotheses);

  RCLCPP_DEBUG(node_->get_logger(), "Assigned probabilities to %zu hypotheses", hypotheses.size());
}

IntentFeatures IntentClassifier::extractFeatures(const vision_msgs::msg::Detection3D & detection)
{
  // PLACEHOLDER: Extract basic features (needs tracking history for velocity)
  IntentFeatures features;

  // Estimate velocity from detection (would normally use tracking history)
  features.velocity = 5.0;  // Placeholder: assume 5 m/s

  // Estimate heading from detection orientation
  features.heading = 0.0;  // Placeholder

  // Distance to intersection (would query map via LaneletHandler)
  features.distance_to_intersection = 50.0;  // Placeholder: 50m

  // Lateral offset from lane centerline (would use LaneletHandler)
  features.lateral_offset = 0.0;  // Placeholder: centered

  // Turn signals (would come from CAN bus or visual detection)
  features.turn_signal_left = false;
  features.turn_signal_right = false;

  // Number of possible future paths (would use LaneletHandler)
  features.num_possible_lanelets = 1;  // Placeholder

  // Time in lane (would track from history)
  features.time_in_lane = 2.0;  // Placeholder: 2 seconds

  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Using placeholder feature extraction");

  return features;
}

double IntentClassifier::computeIntentProbability(Intent intent, const IntentFeatures & features)
{
  double probability = 0.0;

  switch (intent) {
    case Intent::CONTINUE_STRAIGHT:
      // Higher probability if moving straight, no turn signals
      probability = 0.6 * (1.0 - std::abs(features.lateral_offset)) *
                    (!features.turn_signal_left && !features.turn_signal_right ? 1.0 : 0.5);
      break;

    case Intent::TURN_LEFT:
      // Higher probability if turn signal on and near intersection
      probability =
        (features.turn_signal_left ? 0.8 : 0.2) * std::max(0.0, 1.0 - features.distance_to_intersection / 50.0);
      break;

    case Intent::TURN_RIGHT:
      // Higher probability if turn signal on and near intersection
      probability =
        (features.turn_signal_right ? 0.8 : 0.2) * std::max(0.0, 1.0 - features.distance_to_intersection / 50.0);
      break;

    case Intent::LANE_CHANGE_LEFT:
      // Higher probability if lateral offset to left
      probability = std::max(0.0, features.lateral_offset) * (features.turn_signal_left ? 1.0 : 0.5);
      break;

    case Intent::LANE_CHANGE_RIGHT:
      // Higher probability if lateral offset to right
      probability = std::max(0.0, -features.lateral_offset) * (features.turn_signal_right ? 1.0 : 0.5);
      break;

    case Intent::STOP:
      // Higher probability if low velocity
      probability = features.velocity < 1.0 ? 0.7 : 0.1;
      break;

    default:
      probability = 0.1;
      break;
  }

  // Clamp to [0, 1]
  return std::max(0.0, std::min(1.0, probability));
}

void IntentClassifier::normalizeProbabilities(std::vector<TrajectoryHypothesis> & hypotheses)
{
  if (hypotheses.empty()) {
    return;
  }

  // Sum all probabilities
  double total_probability = 0.0;
  for (const auto & hypothesis : hypotheses) {
    total_probability += hypothesis.probability;
  }

  // Avoid division by zero
  if (total_probability < 1e-6) {
    // Assign uniform probabilities
    double uniform_prob = 1.0 / hypotheses.size();
    for (auto & hypothesis : hypotheses) {
      hypothesis.probability = uniform_prob;
    }
    return;
  }

  // Normalize
  for (auto & hypothesis : hypotheses) {
    hypothesis.probability /= total_probability;
  }
}

}  // namespace prediction
