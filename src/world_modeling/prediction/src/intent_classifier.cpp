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

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

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

  // Check if probabilities are already set (from lanelet-based scoring)
  double existing_total = 0.0;
  for (const auto & h : hypotheses) {
    existing_total += h.probability;
  }

  if (existing_total > 1e-6) {
    // Probabilities already set by trajectory predictor (lanelet-based) — just normalize
    normalizeProbabilities(hypotheses);
    RCLCPP_DEBUG(node_->get_logger(), "Using lanelet-based probabilities for %zu hypotheses",
      hypotheses.size());
    return;
  }

  // Fallback: compute from intent features (geometric case)
  for (auto & hypothesis : hypotheses) {
    hypothesis.probability = computeIntentProbability(hypothesis.intent, features);
  }

  normalizeProbabilities(hypotheses);

  RCLCPP_DEBUG(node_->get_logger(), "Assigned feature-based probabilities to %zu hypotheses",
    hypotheses.size());
}

IntentFeatures IntentClassifier::extractFeatures(const vision_msgs::msg::Detection3D & detection)
{
  IntentFeatures features;

  features.velocity = 5.0;
  features.heading = 0.0;
  features.distance_to_intersection = 50.0;
  features.lateral_offset = 0.0;
  features.turn_signal_left = false;
  features.turn_signal_right = false;
  features.num_possible_lanelets = 1;
  features.time_in_lane = 2.0;

  return features;
}

double IntentClassifier::computeIntentProbability(Intent intent, const IntentFeatures & features)
{
  double probability = 0.0;

  switch (intent) {
    case Intent::CONTINUE_STRAIGHT:
      probability = 0.6 * (1.0 - std::abs(features.lateral_offset)) *
                    (!features.turn_signal_left && !features.turn_signal_right ? 1.0 : 0.5);
      break;

    case Intent::TURN_LEFT:
      probability =
        (features.turn_signal_left ? 0.8 : 0.2) * std::max(0.0, 1.0 - features.distance_to_intersection / 50.0);
      break;

    case Intent::TURN_RIGHT:
      probability =
        (features.turn_signal_right ? 0.8 : 0.2) * std::max(0.0, 1.0 - features.distance_to_intersection / 50.0);
      break;

    case Intent::LANE_CHANGE_LEFT:
      probability = std::max(0.0, features.lateral_offset) * (features.turn_signal_left ? 1.0 : 0.5);
      break;

    case Intent::LANE_CHANGE_RIGHT:
      probability = std::max(0.0, -features.lateral_offset) * (features.turn_signal_right ? 1.0 : 0.5);
      break;

    case Intent::STOP:
      probability = features.velocity < 1.0 ? 0.7 : 0.1;
      break;

    default:
      probability = 0.1;
      break;
  }

  return std::max(0.0, std::min(1.0, probability));
}

void IntentClassifier::normalizeProbabilities(std::vector<TrajectoryHypothesis> & hypotheses)
{
  if (hypotheses.empty()) {
    return;
  }

  double total_probability = 0.0;
  for (const auto & hypothesis : hypotheses) {
    total_probability += hypothesis.probability;
  }

  if (total_probability < 1e-6) {
    double uniform_prob = 1.0 / hypotheses.size();
    for (auto & hypothesis : hypotheses) {
      hypothesis.probability = uniform_prob;
    }
    return;
  }

  for (auto & hypothesis : hypotheses) {
    hypothesis.probability /= total_probability;
  }
}

}  // namespace prediction
