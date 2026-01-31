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

#ifndef world_model_markers__WORLD_OBJECTS_MARKERS_NODE_HPP_
#define world_model_markers__WORLD_OBJECTS_MARKERS_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace world_model_markers
{

class WorldObjectsMarkersNode : public rclcpp::Node
{
public:
  WorldObjectsMarkersNode();

private:
  void worldObjectsCallback(const world_model_msgs::msg::WorldObjectArray::SharedPtr msg);

  std_msgs::msg::ColorRGBA getColorForClassId(const std::string & class_id) const;

  rclcpp::Subscription<world_model_msgs::msg::WorldObjectArray>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

  std::string frame_id_;
  double box_alpha_;
  double label_text_height_;
  double prediction_line_width_;
};

}  // namespace world_model_markers

#endif  // world_model_markers__WORLD_OBJECTS_MARKERS_NODE_HPP_
