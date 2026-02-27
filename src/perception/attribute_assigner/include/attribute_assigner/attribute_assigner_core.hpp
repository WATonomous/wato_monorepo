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

#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace wato::perception::attribute_assigner
{

/**
 * @brief Parameters for the attribute assigner core.
 */
struct Params
{
  /// Class IDs recognized as traffic lights (e.g., "traffic_light", "9")
  std::vector<std::string> traffic_light_class_ids;

  /// Class IDs recognized as cars/vehicles (e.g., "car", "2", "truck", "7")
  std::vector<std::string> car_class_ids;

  /// Minimum detection confidence to process a detection
  double min_detection_confidence{0.3};

  /// Traffic light: min saturation and value (0-255) to consider a region "lit"
  double traffic_light_min_saturation{60.0};
  double traffic_light_min_value{80.0};

  /// Car: min brightness (HSV V channel) for brake light detection
  double car_brake_min_brightness{90.0};
  /// Car: amber hue range (OpenCV H 0-180) and min saturation/value for turn/hazard
  double car_amber_hue_lo{12.0};
  double car_amber_hue_hi{35.0};
  double car_amber_min_saturation{80.0};
  double car_amber_min_value{100.0};
};

/**
 * @brief Traffic light state attribute confidences.
 *
 * Represents the confidence scores for each possible traffic light state.
 * Downstream consumers can inspect these to determine the most likely state.
 */
struct TrafficLightAttributes
{
  double green{0.0};  ///< Confidence that the light is green
  double yellow{0.0};  ///< Confidence that the light is yellow
  double red{0.0};  ///< Confidence that the light is red
};

/**
 * @brief Car behavior attribute confidences.
 *
 * Represents the confidence scores for each possible car behavior.
 * Downstream consumers can inspect these to determine detected behaviors.
 */
struct CarAttributes
{
  double turning_left{0.0};  ///< Confidence the car is turning left
  double turning_right{0.0};  ///< Confidence the car is turning right
  double braking{0.0};  ///< Confidence the car is braking
  double hazard_lights{0.0};  ///< Confidence the car has hazard lights on
};

/**
 * @brief Core logic for assigning semantic attributes to 2D detections.
 *
 * Processes a Detection2DArray, identifies traffic lights and cars by their
 * class IDs, and assigns each detection's ObjectHypothesisWithPose results
 * array with attribute hypotheses. The original detection hypothesis is
 * preserved; attributes are appended with a namespaced class_id prefix.
 *
 * Traffic light attributes use the prefix "state:" (e.g., "state:green").
 * Car behavior attributes use the prefix "behavior:" (e.g., "behavior:braking").
 *
 * @note The current implementation uses heuristic-based attribute assignment.
 *       In production, these methods should be replaced with ML-based classifiers
 *       operating on cropped image regions.
 */
class AttributeAssignerCore
{
public:
  AttributeAssignerCore() = delete;

  /**
   * @brief Construct the core with the given parameters.
   * @param params Configuration parameters (class IDs, thresholds)
   */
  explicit AttributeAssignerCore(const Params & params);

  /**
   * @brief Process an entire Detection2DArray using the image and enrich with attributes.
   *
   * Crops the image to each detection bbox, runs color-based traffic light state
   * detection and car signal detection (braking, turn, hazard), and appends
   * attribute hypotheses to each matching detection.
   *
   * @param image The image (e.g. from the camera) in BGR; must match the frame of the detections
   * @param input The input detection array (YOLOv8 COCO-style class IDs: 9=traffic_light, 2=car, 7=truck, 5=bus)
   * @return Enriched Detection2DArray with attribute hypotheses added
   */
  vision_msgs::msg::Detection2DArray process(const cv::Mat & image, const vision_msgs::msg::Detection2DArray & input);

  /**
   * @brief Total detections processed across all calls.
   * @return Total detection count
   */
  uint64_t getProcessedCount() const;

  /**
   * @brief Processing time (ms) for the most recent call.
   * @return Processing time in milliseconds
   */
  double getLastProcessingTimeMs() const;

  /// Attribute class_id prefix for traffic light states
  static constexpr auto kStatePrefix = "state:";
  /// Attribute class_id prefix for car behaviors
  static constexpr auto kBehaviorPrefix = "behavior:";

private:
  /**
   * @brief Check if a detection's best hypothesis matches a traffic light class.
   * @param det The detection to check
   * @return true if the detection is a traffic light
   */
  bool isTrafficLight(const vision_msgs::msg::Detection2D & det) const;

  /**
   * @brief Check if a detection's best hypothesis matches a car/vehicle class.
   * @param det The detection to check
   * @return true if the detection is a car or vehicle
   */
  bool isCar(const vision_msgs::msg::Detection2D & det) const;

  /**
   * @brief Get the class_id of the highest-scoring hypothesis.
   * @param det The detection to inspect
   * @return The class_id string, or empty string if no hypotheses exist
   */
  static std::string getBestClassId(const vision_msgs::msg::Detection2D & det);

  /**
   * @brief Get the score of the highest-scoring hypothesis.
   * @param det The detection to inspect
   * @return The score, or 0.0 if no hypotheses exist
   */
  static double getBestScore(const vision_msgs::msg::Detection2D & det);

  /**
   * @brief Crop image to detection bbox with clamping; returns empty Mat if invalid.
   */
  cv::Mat cropToBbox(const cv::Mat & image, const vision_msgs::msg::Detection2D & det) const;

  /**
   * @brief Classify traffic light state from the cropped image using HSV color detection.
   *
   * Splits the crop into regions (vertical: top/middle/bottom = red/yellow/green).
   * Determines which region is "lit" (high saturation and value), then maps hue to color.
   * Uses integral images for O(1) region mean calculations.
   *
   * @param crop Cropped image (traffic light bbox)
   * @return Attribute confidences for each traffic light state
   */
  TrafficLightAttributes classifyTrafficLightState(const cv::Mat & crop) const;

  /**
   * @brief Classify car behavior from the cropped image: brake lights and turn/hazard signals.
   *
   * Converts crop to HSV once and uses integral images for O(1) region mean calculations.
   * Bottom region: red hue + high S/V -> braking. Left/right regions: amber hue -> turn or hazard.
   *
   * @param crop Cropped image (car/vehicle bbox)
   * @return Attribute confidences for each car behavior
   */
  CarAttributes classifyCarBehavior(const cv::Mat & crop) const;

  /**
   * @brief Append traffic light attribute hypotheses to a detection.
   * @param det The detection to enrich (modified in place)
   * @param attrs The attribute confidences to add
   */
  static void appendTrafficLightHypotheses(vision_msgs::msg::Detection2D & det, const TrafficLightAttributes & attrs);

  /**
   * @brief Append car behavior attribute hypotheses to a detection.
   * @param det The detection to enrich (modified in place)
   * @param attrs The attribute confidences to add
   */
  static void appendCarHypotheses(vision_msgs::msg::Detection2D & det, const CarAttributes & attrs);

  /**
   * @brief Create and return an ObjectHypothesisWithPose.
   * @param class_id The class identifier string
   * @param score The confidence score
   * @return A populated ObjectHypothesisWithPose message
   */
  static vision_msgs::msg::ObjectHypothesisWithPose makeHypothesis(const std::string & class_id, double score);

  Params params_;
  std::unordered_set<std::string> traffic_light_ids_;  ///< Fast lookup for traffic light class IDs
  std::unordered_set<std::string> car_ids_;  ///< Fast lookup for car class IDs

  uint64_t processed_count_{0};  ///< Total detections processed
  double last_processing_time_ms_{0.0};  ///< Processing time for last call (ms)
};

}  // namespace wato::perception::attribute_assigner
