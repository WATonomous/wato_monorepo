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

#ifndef world_model_markers__DYNAMIC_OBJECTS_MARKERS_NODE_HPP_
#define world_model_markers__DYNAMIC_OBJECTS_MARKERS_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "world_model_msgs/msg/dynamic_object.hpp"
#include "world_model_msgs/msg/dynamic_object_array.hpp"

namespace world_model_markers
{

/**
 * @brief Converts DynamicObjectArray messages to visualization markers.
 *
 * Subscribes to world-model-processed dynamic objects and publishes
 * bounding box, label, history trail, and prediction line markers for each object.
 */
class DynamicObjectsMarkersNode : public rclcpp::Node
{
public:
  DynamicObjectsMarkersNode();

private:
  /**
   * @brief Callback for incoming DynamicObjectArray messages.
   *
   * Clears previous markers, then creates a bounding box, label, history trail,
   * and prediction line strip for each tracked object in the message.
   *
   * @param msg Array of dynamic objects with pose, history, and prediction data.
   */
  void dynamicObjectsCallback(const world_model_msgs::msg::DynamicObjectArray::SharedPtr msg);

  /**
   * @brief Maps an entity type enum value to a visualization color.
   *
   * @param entity_type DynamicObject entity type constant (e.g. TYPE_CAR).
   * @return RGBA color for the given type; gray for TYPE_UNKNOWN.
   */
  std_msgs::msg::ColorRGBA getColorForEntityType(uint8_t entity_type) const;

  /**
   * @brief Maps an entity type enum value to a human-readable string.
   *
   * @param entity_type DynamicObject entity type constant.
   * @return Display name (e.g. "CAR", "HUMAN").
   */
  std::string getEntityTypeName(uint8_t entity_type) const;

  rclcpp::Subscription<world_model_msgs::msg::DynamicObjectArray>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

  std::string frame_id_;
  double box_alpha_;
  double label_text_height_;
  double history_line_width_;
  double prediction_line_width_;
};

}  // namespace world_model_markers

#endif  // world_model_markers__DYNAMIC_OBJECTS_MARKERS_NODE_HPP_
