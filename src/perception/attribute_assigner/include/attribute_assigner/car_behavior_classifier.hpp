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
 * @brief Classifies car behavior (braking, turning, hazard) using HSV blob analysis.
 *
 * Detects red blobs for brake lights and amber blobs for turn/hazard signals.
 * Uses symmetric pair detection for braking confidence and per-side amber area
 * analysis for turn signal vs hazard classification.
 */
class CarBehaviorClassifier : public AttributeClassifier
{
public:
  /// Attribute class_id prefix for car behaviors
  static constexpr auto kPrefix = "behavior:";

  /**
   * @brief Parameters for car behavior HSV classification and 3D sizing.
   */
  struct Params
  {
    /// Class IDs recognized as cars/vehicles (e.g., "car", "2", "truck", "7", "bus", "5")
    std::vector<std::string> class_ids;

    /// Min brightness and saturation (HSV) for brake light detection
    double brake_min_brightness{90.0};
    double brake_min_saturation{40.0};

    /// Red hue range for brake lights (same wrapping as traffic lights)
    double red_hue_lo{10.0};
    double red_hue_hi{170.0};

    /// Amber hue range (OpenCV H 0-180) and min saturation/value for turn/hazard
    double amber_hue_lo{12.0};
    double amber_hue_hi{35.0};
    double amber_min_saturation{80.0};
    double amber_min_value{100.0};

    /// Vehicle subtype class IDs (for 3D box sizing)
    std::vector<std::string> truck_class_ids;
    std::vector<std::string> bus_class_ids;

    /// 3D dimensions per vehicle subtype (meters)
    double car_width{1.8};
    double car_height{1.8};
    double car_length{6.0};
    double truck_width{2.5};
    double truck_height{3.5};
    double truck_length{8.0};
    double bus_width{2.5};
    double bus_height{3.2};
    double bus_length{12.0};

    /// Fallback depth for all vehicles (meters)
    double assumed_depth{20.0};
  };

  explicit CarBehaviorClassifier(const Params & params);

  bool matches(const vision_msgs::msg::Detection2D & det) const override;
  bool matchesClassId(const std::string & class_id) const override;
  void classify(const cv::Mat & crop, vision_msgs::msg::Detection2D & det) const override;
  BoxParams3D get3DBoxParams(const vision_msgs::msg::Detection2D & det) const override;
  MarkerColor getMarkerColor(const vision_msgs::msg::Detection2D & det) const override;

private:
  Params params_;
  std::unordered_set<std::string> ids_;
  std::unordered_set<std::string> truck_ids_;
  std::unordered_set<std::string> bus_ids_;
};

}  // namespace wato::perception::attribute_assigner
