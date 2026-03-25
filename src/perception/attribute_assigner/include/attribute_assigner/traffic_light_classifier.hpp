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

#include "attribute_assigner/attribute_classifier.hpp"

namespace wato::perception::attribute_assigner
{

/**
 * @brief Classifies traffic light state (red/yellow/green) using HSV color analysis.
 *
 * Analyzes a center strip of the traffic light crop, counts bright pixels per hue bucket,
 * and produces confidence scores as pixel ratios. Defaults to red when insufficient
 * bright pixels are found (safety-first for autonomous vehicles).
 */
class TrafficLightClassifier : public AttributeClassifier
{
public:
  /// Attribute class_id prefix for traffic light states
  static constexpr auto kPrefix = "state:";

  /**
   * @brief Parameters for traffic light HSV classification.
   */
  struct Params
  {
    /// Class IDs recognized as traffic lights (e.g., "traffic light", "9")
    std::vector<std::string> class_ids;

    /// Min saturation and value (0-255) to consider a region "lit"
    double min_saturation{60.0};
    double min_value{80.0};

    /// Fraction of width/height to ignore on each side (0.0-0.45)
    double strip_margin{0.20};

    /// Hue boundaries (OpenCV H 0-180)
    /// Red wraps around: H <= red_hue_lo OR H >= red_hue_hi
    double red_hue_lo{10.0};
    double red_hue_hi{170.0};
    /// Yellow: red_hue_lo < H <= yellow_hue_hi
    double yellow_hue_hi{35.0};
    /// Green: yellow_hue_hi < H < green_hue_hi
    double green_hue_hi{85.0};

    /// 3D box: assumed depth fallback (meters)
    double assumed_depth{30.0};
    /// 3D box: physical dimensions (meters)
    double box_width{0.3};
    double box_height{1.0};
    double box_length{0.3};
  };

  explicit TrafficLightClassifier(const Params & params);

  bool matches(const vision_msgs::msg::Detection2D & det) const override;
  bool matchesClassId(const std::string & class_id) const override;
  void classify(const cv::Mat & crop, vision_msgs::msg::Detection2D & det) const override;
  BoxParams3D get3DBoxParams(const vision_msgs::msg::Detection2D & det) const override;
  MarkerColor getMarkerColor(const vision_msgs::msg::Detection2D & det) const override;

private:
  Params params_;
  std::unordered_set<std::string> ids_;
};

}  // namespace wato::perception::attribute_assigner
