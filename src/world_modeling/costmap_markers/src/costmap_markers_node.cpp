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

#include "costmap_markers/costmap_markers_node.hpp"

#include <functional>
#include <memory>
#include <vector>

namespace costmap_markers
{

CostmapMarkersNode::CostmapMarkersNode()
: Node("costmap_markers_node")
{
  this->declare_parameter("frame_id", "base_footprint");
  this->declare_parameter("footprint_color", std::vector<double>{0.0, 1.0, 0.0, 0.8});
  this->declare_parameter("footprint_line_width", 0.05);

  frame_id_ = this->get_parameter("frame_id").as_string();
  footprint_color_ = this->get_parameter("footprint_color").as_double_array();
  footprint_line_width_ = this->get_parameter("footprint_line_width").as_double();

  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  subscription_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "footprint", 10, std::bind(&CostmapMarkersNode::footprintCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "CostmapMarkersNode initialized");
}

void CostmapMarkersNode::footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  if (msg->polygon.points.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = msg->header.stamp;
  marker.header.frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;
  marker.ns = "costmap_footprint";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = footprint_line_width_;
  marker.color.r = footprint_color_.size() > 0 ? static_cast<float>(footprint_color_[0]) : 0.0f;
  marker.color.g = footprint_color_.size() > 1 ? static_cast<float>(footprint_color_[1]) : 1.0f;
  marker.color.b = footprint_color_.size() > 2 ? static_cast<float>(footprint_color_[2]) : 0.0f;
  marker.color.a = footprint_color_.size() > 3 ? static_cast<float>(footprint_color_[3]) : 0.8f;
  marker.pose.orientation.w = 1.0;

  for (const auto & pt : msg->polygon.points) {
    geometry_msgs::msg::Point p;
    p.x = static_cast<double>(pt.x);
    p.y = static_cast<double>(pt.y);
    p.z = static_cast<double>(pt.z);
    marker.points.push_back(p);
  }

  // Close the loop
  if (!msg->polygon.points.empty()) {
    geometry_msgs::msg::Point p;
    p.x = static_cast<double>(msg->polygon.points.front().x);
    p.y = static_cast<double>(msg->polygon.points.front().y);
    p.z = static_cast<double>(msg->polygon.points.front().z);
    marker.points.push_back(p);
  }

  marker_array.markers.push_back(marker);
  publisher_->publish(marker_array);
}

}  // namespace costmap_markers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<costmap_markers::CostmapMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
