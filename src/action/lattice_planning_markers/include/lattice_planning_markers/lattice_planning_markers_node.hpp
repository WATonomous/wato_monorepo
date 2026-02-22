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
#include <vector>

#include "lattice_planning_msgs/msg/path_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lattice_planning_markers
{

class LatticePlanningMarkersNode : public rclcpp::Node
{
public:
  explicit LatticePlanningMarkersNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void availablePathsCallback(const lattice_planning_msgs::msg::PathArray::SharedPtr msg);

  visualization_msgs::msg::MarkerArray pathToLineStripMarkers(
    const nav_msgs::msg::Path & path,
    const std::string & ns,
    int32_t id,
    float r,
    float g,
    float b,
    float a,
    float line_width) const;

  visualization_msgs::msg::MarkerArray pathArrayToMarkers(
    const lattice_planning_msgs::msg::PathArray & path_array) const;

  visualization_msgs::msg::Marker makeDeleteAllMarker(const std_msgs::msg::Header & header) const;

  // Subscription topics names
  std::string path_topic_, available_paths_topic_;

  // Pulisher topic names
  std::string path_markers_topic_, available_paths_markers_topic_;

  // Parameters (appearance)
  double path_line_width_;
  double available_path_line_width_;
  double available_paths_alpha_;
  int max_available_paths_;
  bool publish_deleteall_each_update_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<lattice_planning_msgs::msg::PathArray>::SharedPtr available_paths_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr available_paths_markers_pub_;
};

}  // namespace lattice_planning_markers
