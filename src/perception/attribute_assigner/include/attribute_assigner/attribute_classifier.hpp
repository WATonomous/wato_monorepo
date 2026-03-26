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

#include <opencv2/core/mat.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace wato::perception::attribute_assigner
{

/**
 * @brief Physical 3D box parameters for a detection.
 *
 * Returned by classifiers to provide type-specific 3D sizing and depth estimation.
 */
struct BoxParams3D
{
  double width{0.0};  // Physical width (meters), used for depth estimation if use_width_for_depth
  double height{0.0};  // Physical height (meters), used for depth estimation if !use_width_for_depth
  double length{0.0};  // Physical length/depth (meters)
  double assumed_depth{20.0};  // Fallback depth when estimation is out of range (meters)
  bool use_width_for_depth{true};  // If true, estimate depth from width; else from height
};

/**
 * @brief RGBA color for marker visualization.
 */
struct MarkerColor
{
  float r{1.0f};
  float g{1.0f};
  float b{1.0f};
  float a{0.5f};
};

/**
 * @brief Abstract interface for attribute classifiers.
 *
 * Each classifier is responsible for:
 * 1. Determining whether it handles a given detection (via matches())
 * 2. Classifying the cropped image and appending attribute hypotheses (via classify())
 * 3. Providing 3D box parameters for depth estimation and sizing (via get3DBoxParams())
 * 4. Providing marker colors based on classified attributes (via getMarkerColor())
 *
 * To add a new attribute classifier:
 * 1. Create a new class that inherits from AttributeClassifier
 * 2. Implement matches() to check if the detection's class_id is relevant
 * 3. Implement classify() to analyze the crop and append hypotheses to the detection
 * 4. Implement get3DBoxParams() and getMarkerColor() for visualization
 * 5. Register the classifier via core_->addClassifier() in the node's declareParameters()
 */
class AttributeClassifier
{
public:
  virtual ~AttributeClassifier() = default;

  /**
   * @brief Check if this classifier handles the given detection.
   * @param det The detection to check
   * @return true if this classifier should process the detection
   */
  virtual bool matches(const vision_msgs::msg::Detection2D & det) const = 0;

  /**
   * @brief Check if a single class_id string matches this classifier.
   * @param class_id The class identifier to check
   * @return true if the class_id is handled by this classifier
   */
  virtual bool matchesClassId(const std::string & class_id) const = 0;

  /**
   * @brief Classify the cropped image and append attribute hypotheses to the detection.
   * @param crop The cropped image (BGR, from the detection's bounding box)
   * @param det The detection to enrich (modified in place)
   */
  virtual void classify(const cv::Mat & crop, vision_msgs::msg::Detection2D & det) const = 0;

  /**
   * @brief Get 3D box parameters for a detection.
   *
   * Returns physical dimensions and depth estimation strategy. Classifiers may
   * inspect the detection's results to determine subtypes (e.g., truck vs car).
   *
   * @param det The detection (may contain subtype hypotheses)
   * @return BoxParams3D with physical dimensions and depth strategy
   */
  virtual BoxParams3D get3DBoxParams(const vision_msgs::msg::Detection2D & det) const = 0;

  /**
   * @brief Get marker color based on the detection's classified attributes.
   *
   * Inspects the enriched hypotheses (e.g., state:red, behavior:braking) and
   * returns an appropriate visualization color.
   *
   * @param det The enriched detection with attribute hypotheses
   * @return MarkerColor with RGBA values
   */
  virtual MarkerColor getMarkerColor(const vision_msgs::msg::Detection2D & det) const = 0;

protected:
  static vision_msgs::msg::ObjectHypothesisWithPose makeHypothesis(const std::string & class_id, double score)
  {
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = class_id;
    hyp.hypothesis.score = score;
    return hyp;
  }
};

}  // namespace wato::perception::attribute_assigner
