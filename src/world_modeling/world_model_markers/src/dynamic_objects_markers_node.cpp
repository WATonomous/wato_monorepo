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

#include "world_model_markers/dynamic_objects_markers_node.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "lanelet_markers/marker_utils.hpp"

namespace world_model_markers
{

DynamicObjectsMarkersNode::DynamicObjectsMarkersNode()
: Node("dynamic_objects_markers_node")
{
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("box_alpha", 0.6);
  this->declare_parameter("label_text_height", 0.5);
  this->declare_parameter("history_line_width", 0.1);
  this->declare_parameter("prediction_line_width", 0.08);

  frame_id_ = this->get_parameter("frame_id").as_string();
  box_alpha_ = this->get_parameter("box_alpha").as_double();
  label_text_height_ = this->get_parameter("label_text_height").as_double();
  history_line_width_ = this->get_parameter("history_line_width").as_double();
  prediction_line_width_ = this->get_parameter("prediction_line_width").as_double();

  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  subscription_ = this->create_subscription<world_model_msgs::msg::DynamicObjectArray>(
    "dynamic_objects", 10,
    std::bind(&DynamicObjectsMarkersNode::dynamicObjectsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "DynamicObjectsMarkersNode initialized");
}

std_msgs::msg::ColorRGBA DynamicObjectsMarkersNode::getColorForEntityType(
  uint8_t entity_type) const
{
  using DynamicObject = world_model_msgs::msg::DynamicObject;

  switch (entity_type) {
    case DynamicObject::TYPE_CAR:
      return lanelet_markers::makeColor(0.2f, 0.4f, 1.0f, static_cast<float>(box_alpha_));
    case DynamicObject::TYPE_HUMAN:
      return lanelet_markers::makeColor(1.0f, 1.0f, 0.0f, static_cast<float>(box_alpha_));
    case DynamicObject::TYPE_BICYCLE:
      return lanelet_markers::makeColor(0.0f, 1.0f, 1.0f, static_cast<float>(box_alpha_));
    case DynamicObject::TYPE_MOTORCYCLE:
      return lanelet_markers::makeColor(1.0f, 0.0f, 1.0f, static_cast<float>(box_alpha_));
    case DynamicObject::TYPE_TRAFFIC_LIGHT:
      return lanelet_markers::makeColor(1.0f, 1.0f, 1.0f, static_cast<float>(box_alpha_));
    default:  // TYPE_UNKNOWN
      return lanelet_markers::makeColor(0.6f, 0.6f, 0.6f, static_cast<float>(box_alpha_));
  }
}

std::string DynamicObjectsMarkersNode::getEntityTypeName(uint8_t entity_type) const
{
  using DynamicObject = world_model_msgs::msg::DynamicObject;

  switch (entity_type) {
    case DynamicObject::TYPE_CAR:
      return "CAR";
    case DynamicObject::TYPE_HUMAN:
      return "HUMAN";
    case DynamicObject::TYPE_BICYCLE:
      return "BICYCLE";
    case DynamicObject::TYPE_MOTORCYCLE:
      return "MOTORCYCLE";
    case DynamicObject::TYPE_TRAFFIC_LIGHT:
      return "TRAFFIC_LIGHT";
    default:
      return "UNKNOWN";
  }
}

void DynamicObjectsMarkersNode::dynamicObjectsCallback(
  const world_model_msgs::msg::DynamicObjectArray::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int32_t marker_id = 0;

  std::string frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;

  auto stamp = msg->header.stamp;
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    stamp = this->get_clock()->now();
  }

  // Delete all previous markers
  std::vector<std::string> namespaces = {
    "do_boxes", "do_labels", "do_history", "do_predictions"};
  for (const auto & ns : namespaces) {
    auto delete_marker = lanelet_markers::createDeleteAllMarker(ns, frame_id);
    delete_marker.header.stamp = stamp;
    marker_array.markers.push_back(delete_marker);
  }

  for (const auto & obj : msg->objects) {
    auto color = getColorForEntityType(obj.entity_type);
    std::string type_name = getEntityTypeName(obj.entity_type);

    // Bounding box (CUBE)
    auto box_marker = lanelet_markers::createCubeMarker(
      "do_boxes", marker_id++, frame_id, obj.pose, obj.size, color);
    box_marker.header.stamp = stamp;
    marker_array.markers.push_back(box_marker);

    // ID label (TEXT_VIEW_FACING) above the box
    std::string label_text = std::to_string(obj.id) + "\n" + type_name;
    if (obj.lanelet_id >= 0) {
      label_text += "\nL:" + std::to_string(obj.lanelet_id);
    }

    geometry_msgs::msg::Point label_pos = obj.pose.position;
    label_pos.z += obj.size.z / 2.0 + label_text_height_;

    auto label_color = lanelet_markers::makeColor(1.0f, 1.0f, 1.0f, 0.9f);
    auto label_marker = lanelet_markers::createTextMarker(
      "do_labels", marker_id++, frame_id, label_pos, label_text, label_color,
      label_text_height_);
    label_marker.header.stamp = stamp;
    marker_array.markers.push_back(label_marker);

    // History trail (LINE_STRIP from history poses)
    if (obj.history.size() >= 2) {
      std::vector<geometry_msgs::msg::Point> history_points;
      // Current position first
      history_points.push_back(obj.pose.position);
      for (const auto & pose_stamped : obj.history) {
        history_points.push_back(pose_stamped.pose.position);
      }

      auto history_color = color;
      history_color.a = 0.4f;

      auto history_marker = lanelet_markers::createLineStripMarker(
        "do_history", marker_id++, frame_id, history_points, history_color,
        history_line_width_);
      history_marker.header.stamp = stamp;
      marker_array.markers.push_back(history_marker);
    }

    // Predictions (LINE_STRIP per path)
    for (const auto & prediction : obj.predictions) {
      if (prediction.poses.empty()) {
        continue;
      }

      std::vector<geometry_msgs::msg::Point> pred_points;
      // Start from the object's current position
      pred_points.push_back(obj.pose.position);
      for (const auto & pose_stamped : prediction.poses) {
        pred_points.push_back(pose_stamped.pose.position);
      }

      auto pred_color = color;
      pred_color.a = static_cast<float>(prediction.conf * 0.8);

      auto pred_marker = lanelet_markers::createLineStripMarker(
        "do_predictions", marker_id++, frame_id, pred_points, pred_color,
        prediction_line_width_);
      pred_marker.header.stamp = stamp;
      marker_array.markers.push_back(pred_marker);
    }
  }

  publisher_->publish(marker_array);
}

}  // namespace world_model_markers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<world_model_markers::DynamicObjectsMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
