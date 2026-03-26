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

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "attribute_assigner/attribute_classifier.hpp"

namespace wato::perception::attribute_assigner
{

/**
 * @brief Camera intrinsics needed for 3D projection (no ROS dependencies).
 */
struct CameraIntrinsics
{
  double fx{0.0};  // Focal length x (pixels)
  double fy{0.0};  // Focal length y (pixels)
  double cx{0.0};  // Principal point x (pixels)
  double cy{0.0};  // Principal point y (pixels)
};

/**
 * @brief Result of projecting a 2D detection to 3D (in camera frame).
 */
struct Projection3D
{
  double x{0.0};  // X position in camera frame (meters)
  double y{0.0};  // Y position in camera frame (meters)
  double z{0.0};  // Z position in camera frame (meters)
  double size_x{0.0};  // Clamped width (meters)
  double size_y{0.0};  // Clamped length (meters)
  double size_z{0.0};  // Clamped height (meters)
};

/**
 * @brief Core logic for assigning semantic attributes to 2D detections.
 *
 * Uses a plugin-based classifier system: each registered AttributeClassifier
 * checks if it handles a detection and, if so, classifies the crop and appends
 * attribute hypotheses. New classifiers can be added by implementing
 * AttributeClassifier and registering them via addClassifier().
 *
 * Built-in classifiers:
 * - TrafficLightClassifier: HSV-based state detection (state:red, state:yellow, state:green)
 * - CarBehaviorClassifier: blob-based signal detection (behavior:braking, behavior:turning_*, behavior:hazard_lights)
 */
class AttributeAssignerCore
{
public:
  /**
   * @brief Construct the core with a minimum detection confidence threshold.
   * @param min_detection_confidence Detections below this score are passed through unmodified
   */
  explicit AttributeAssignerCore(double min_detection_confidence);

  /**
   * @brief Register an attribute classifier.
   *
   * Classifiers are evaluated in registration order; first match wins.
   *
   * @param classifier The classifier to register (ownership transferred)
   */
  void addClassifier(std::unique_ptr<AttributeClassifier> classifier);

  /**
   * @brief Process an entire Detection2DArray using the image and enrich with attributes.
   *
   * For each detection, iterates registered classifiers. The first matching classifier
   * crops the image and appends attribute hypotheses.
   *
   * @param image The image in BGR; must match the frame of the detections
   * @param input The input detection array
   * @return Enriched Detection2DArray with attribute hypotheses added
   */
  vision_msgs::msg::Detection2DArray process(const cv::Mat & image, const vision_msgs::msg::Detection2DArray & input);

  uint64_t getProcessedCount() const;
  double getLastProcessingTimeMs() const;

  double getMinDetectionConfidence() const
  {
    return min_detection_confidence_;
  }

  /**
   * @brief Get the registered classifiers (read-only).
   */
  const std::vector<std::unique_ptr<AttributeClassifier>> & getClassifiers() const
  {
    return classifiers_;
  }

  /**
   * @brief Find the first classifier that matches a detection.
   * @param det The detection to check
   * @return Pointer to the matching classifier, or nullptr if none match
   */
  const AttributeClassifier * findClassifier(const vision_msgs::msg::Detection2D & det) const;

  /**
   * @brief Find the first classifier that matches a class_id string.
   * @param class_id The class identifier to check
   * @return Pointer to the matching classifier, or nullptr if none match
   */
  const AttributeClassifier * findClassifierByClassId(const std::string & class_id) const;

  static std::string getBestClassId(const vision_msgs::msg::Detection2D & det);
  static double getBestScore(const vision_msgs::msg::Detection2D & det);

  cv::Mat cropToBbox(const cv::Mat & image, const vision_msgs::msg::Detection2D & det) const;

  /**
   * @brief Project a 2D detection to 3D using camera intrinsics and box parameters.
   *
   * Pure math: estimates depth from known physical size and pixel footprint,
   * clamps 3D dimensions to not exceed the 2D bbox projection, and unprojects
   * the bbox center to a 3D ray scaled to the estimated depth.
   *
   * @param intrinsics Camera intrinsics (fx, fy, cx, cy)
   * @param det The 2D detection (bbox center and size in pixels)
   * @param box_params Physical 3D box parameters from the classifier
   * @return Projection3D with 3D position (in camera frame) and clamped dimensions
   */
  static Projection3D projectTo3D(
    const CameraIntrinsics & intrinsics, const vision_msgs::msg::Detection2D & det, const BoxParams3D & box_params);

private:
  double min_detection_confidence_;

  /// Registered attribute classifiers (processed in order; first match wins)
  std::vector<std::unique_ptr<AttributeClassifier>> classifiers_;

  uint64_t processed_count_{0};
  double last_processing_time_ms_{0.0};
};

}  // namespace wato::perception::attribute_assigner
