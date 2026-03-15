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

/**
 * @file intent_classifier.hpp
 * @brief Assigns probabilities to trajectory hypotheses based on intent.
 */

#ifndef PREDICTION__INTENT_CLASSIFIER_HPP_
#define PREDICTION__INTENT_CLASSIFIER_HPP_

#include <memory>
#include <vector>

#include "prediction/trajectory_predictor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection3_d.hpp"

namespace prediction
{

/**
 * @brief Features used for intent classification
 */
struct IntentFeatures
{
  double velocity;  // Current velocity magnitude
  double heading;  // Current heading
  double distance_to_intersection;  // Distance to nearest intersection
  double lateral_offset;  // Offset from lane centerline
  bool turn_signal_left;  // Turn signal observations
  bool turn_signal_right;
  int num_possible_lanelets;  // Number of possible future lanelets
  double time_in_lane;  // Time spent in current lane
};

/**
 * @brief Configuration for the intent classifier
 */
struct IntentClassifierConfig
{
  // Feature defaults
  double default_velocity = 5.0;
  double default_distance_to_intersection = 50.0;
  double default_time_in_lane = 2.0;

  // Probability model
  double straight_base_probability = 0.6;
  double turn_signal_discount = 0.5;
  double turn_signal_presence_weight = 0.8;
  double turn_signal_absence_weight = 0.2;
  double intersection_normalization = 50.0;
  double stop_velocity_threshold = 1.0;
  double stop_high_probability = 0.7;
  double stop_low_probability = 0.1;
  double default_fallback_probability = 0.1;
};

/**
 * @brief Assigns probabilities to trajectory hypotheses based on intent
 * * Uses learned or rule-based classifier to estimate the probability of each
 * trajectory hypothesis based on object state and context.
 */
class IntentClassifier
{
public:
  /**
   * @brief Construct a new Intent Classifier
   * @param node ROS node pointer for logging
   * @param config Configuration parameters
   */
  explicit IntentClassifier(rclcpp_lifecycle::LifecycleNode * node, const IntentClassifierConfig & config = {});

  /**
   * @brief Assign probabilities to trajectory hypotheses
   * @param detection Tracked object detection
   * @param hypotheses Trajectory hypotheses to classify
   * @param features Intent features for classification
   */
  void assignProbabilities(
    const vision_msgs::msg::Detection3D & detection,
    std::vector<TrajectoryHypothesis> & hypotheses,
    const IntentFeatures & features);

  /**
   * @brief Extract features for intent classification
   * @param detection Tracked object detection
   * @return IntentFeatures Feature vector for classification
   */
  IntentFeatures extractFeatures(const vision_msgs::msg::Detection3D & detection);

private:
  /**
   * @brief Compute probability for a specific intent
   * @param intent Intent type
   * @param features Feature vector
   * @return Probability value [0, 1]
   */
  double computeIntentProbability(Intent intent, const IntentFeatures & features);

  /**
   * @brief Normalize probabilities so they sum to 1.0
   * @param hypotheses Trajectory hypotheses with probabilities
   */
  void normalizeProbabilities(std::vector<TrajectoryHypothesis> & hypotheses);

  rclcpp_lifecycle::LifecycleNode * node_;
  IntentClassifierConfig config_;
};

}  // namespace prediction

#endif  // PREDICTION__INTENT_CLASSIFIER_HPP_
