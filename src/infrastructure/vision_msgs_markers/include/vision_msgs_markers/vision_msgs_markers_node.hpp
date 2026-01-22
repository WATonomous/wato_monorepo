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

#ifndef VISION_MSGS_MARKERS__VISION_MSGS_MARKERS_NODE_HPP_
#define VISION_MSGS_MARKERS__VISION_MSGS_MARKERS_NODE_HPP_

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace vision_msgs_markers
{

class VisionMsgsMarkersNode : public rclcpp::Node
{
public:
  VisionMsgsMarkersNode();

  static constexpr auto kInputTopic = "detections_in";
  static constexpr auto kOutputTopic = "markers_out";

private:
  void detectionsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  visualization_msgs::msg::Marker createBoxMarker(
    const vision_msgs::msg::Detection3D & detection, const std_msgs::msg::Header & header, int id);

  visualization_msgs::msg::Marker createFillMarker(
    const vision_msgs::msg::Detection3D & detection, const std_msgs::msg::Header & header, int id);

  visualization_msgs::msg::Marker createTextMarker(
    const vision_msgs::msg::Detection3D & detection, const std_msgs::msg::Header & header, int id);

  std::array<float, 4> getColorForClassId(const std::string & class_id);

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  double marker_alpha_;
  double fill_alpha_;
  double text_scale_;
  double line_width_;

  std::unordered_map<std::string, std::array<float, 4>> color_cache_;
  std::vector<std::array<float, 3>> color_palette_;
};

}  // namespace vision_msgs_markers

#endif  // VISION_MSGS_MARKERS__VISION_MSGS_MARKERS_NODE_HPP_
