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

IntentClassifier::IntentClassifier(rclcpp_lifecycle::LifecycleNode * node, const IntentClassifierConfig & config)
: node_(node)
, config_(config)
{
  weights_.velocity_weight = config_.velocity_weight;
  weights_.heading_weight = config_.heading_weight;
  weights_.intersection_weight = config_.intersection_weight;
  weights_.lateral_offset_weight = config_.lateral_offset_weight;
  weights_.turn_signal_weight = config_.turn_signal_weight;

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
    RCLCPP_DEBUG(node_->get_logger(), "Using lanelet-based probabilities for %zu hypotheses", hypotheses.size());
    return;
  }

  // Fallback: compute from intent features (geometric case)
  for (auto & hypothesis : hypotheses) {
    hypothesis.probability = computeIntentProbability(hypothesis.intent, features);
  }

  normalizeProbabilities(hypotheses);

  RCLCPP_DEBUG(node_->get_logger(), "Assigned feature-based probabilities to %zu hypotheses", hypotheses.size());
}

IntentFeatures IntentClassifier::extractFeatures(const vision_msgs::msg::Detection3D & detection)
{
  IntentFeatures features;

  features.velocity = config_.default_velocity;
  features.heading = 0.0;
  features.distance_to_intersection = config_.default_distance_to_intersection;
  features.lateral_offset = 0.0;
  features.turn_signal_left = false;
  features.turn_signal_right = false;
  features.num_possible_lanelets = 1;
  features.time_in_lane = config_.default_time_in_lane;

  return features;
}

double IntentClassifier::computeIntentProbability(Intent intent, const IntentFeatures & features)
{
  double probability = 0.0;

  switch (intent) {
    case Intent::CONTINUE_STRAIGHT:
      probability = config_.straight_base_probability * (1.0 - std::abs(features.lateral_offset)) *
                    (!features.turn_signal_left && !features.turn_signal_right ? 1.0 : config_.turn_signal_discount);
      break;

    case Intent::TURN_LEFT:
      probability =
        (features.turn_signal_left ? config_.turn_signal_presence_weight : config_.turn_signal_absence_weight) *
        std::max(0.0, 1.0 - features.distance_to_intersection / config_.intersection_normalization);
      break;

    case Intent::TURN_RIGHT:
      probability =
        (features.turn_signal_right ? config_.turn_signal_presence_weight : config_.turn_signal_absence_weight) *
        std::max(0.0, 1.0 - features.distance_to_intersection / config_.intersection_normalization);
      break;

    case Intent::LANE_CHANGE_LEFT:
      probability =
        std::max(0.0, features.lateral_offset) * (features.turn_signal_left ? 1.0 : config_.turn_signal_discount);
      break;

    case Intent::LANE_CHANGE_RIGHT:
      probability =
        std::max(0.0, -features.lateral_offset) * (features.turn_signal_right ? 1.0 : config_.turn_signal_discount);
      break;

    case Intent::STOP:
      probability = features.velocity < config_.stop_velocity_threshold ? config_.stop_high_probability
                                                                        : config_.stop_low_probability;
      break;

    default:
      probability = config_.default_fallback_probability;
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
