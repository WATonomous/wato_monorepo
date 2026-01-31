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

#include "world_model_markers/world_objects_markers_node.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "lanelet_markers/marker_utils.hpp"

namespace world_model_markers
{

WorldObjectsMarkersNode::WorldObjectsMarkersNode()
: Node("world_objects_markers_node")
{
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("box_alpha", 0.6);
  this->declare_parameter("label_text_height", 0.5);
  this->declare_parameter("prediction_line_width", 0.08);

  frame_id_ = this->get_parameter("frame_id").as_string();
  box_alpha_ = this->get_parameter("box_alpha").as_double();
  label_text_height_ = this->get_parameter("label_text_height").as_double();
  prediction_line_width_ = this->get_parameter("prediction_line_width").as_double();

  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  subscription_ = this->create_subscription<world_model_msgs::msg::WorldObjectArray>(
    "world_objects", 10,
    std::bind(&WorldObjectsMarkersNode::worldObjectsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "WorldObjectsMarkersNode initialized");
}

std_msgs::msg::ColorRGBA WorldObjectsMarkersNode::getColorForClassId(
  const std::string & class_id) const
{
  if (class_id == "car" || class_id == "Car" || class_id == "CAR") {
    return lanelet_markers::makeColor(0.2f, 0.4f, 1.0f, static_cast<float>(box_alpha_));
  } else if (class_id == "human" || class_id == "Human" || class_id == "HUMAN" ||
    class_id == "pedestrian" || class_id == "Pedestrian" || class_id == "PEDESTRIAN")
  {
    return lanelet_markers::makeColor(1.0f, 1.0f, 0.0f, static_cast<float>(box_alpha_));
  } else if (class_id == "bicycle" || class_id == "Bicycle" || class_id == "BICYCLE") {
    return lanelet_markers::makeColor(0.0f, 1.0f, 1.0f, static_cast<float>(box_alpha_));
  } else if (class_id == "motorcycle" || class_id == "Motorcycle" || class_id == "MOTORCYCLE") {
    return lanelet_markers::makeColor(1.0f, 0.0f, 1.0f, static_cast<float>(box_alpha_));
  } else if (class_id == "traffic_light" || class_id == "TrafficLight" ||
    class_id == "TRAFFIC_LIGHT")
  {
    return lanelet_markers::makeColor(1.0f, 1.0f, 1.0f, static_cast<float>(box_alpha_));
  }
  // UNKNOWN / default
  return lanelet_markers::makeColor(0.6f, 0.6f, 0.6f, static_cast<float>(box_alpha_));
}

void WorldObjectsMarkersNode::worldObjectsCallback(
  const world_model_msgs::msg::WorldObjectArray::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int32_t marker_id = 0;

  std::string frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;

  auto stamp = msg->header.stamp;
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    stamp = this->get_clock()->now();
  }

  // Delete all previous markers
  std::vector<std::string> namespaces = {"wo_boxes", "wo_labels", "wo_predictions"};
  for (const auto & ns : namespaces) {
    auto delete_marker = lanelet_markers::createDeleteAllMarker(ns, frame_id);
    delete_marker.header.stamp = stamp;
    marker_array.markers.push_back(delete_marker);
  }

  for (const auto & world_obj : msg->objects) {
    const auto & det = world_obj.detection;

    // Get class ID from detection results
    std::string class_id = "unknown";
    if (!det.results.empty()) {
      class_id = det.results[0].hypothesis.class_id;
    }

    auto color = getColorForClassId(class_id);

    // Bounding box (CUBE)
    auto box_marker = lanelet_markers::createCubeMarker(
      "wo_boxes", marker_id++, frame_id, det.bbox.center, det.bbox.size, color);
    box_marker.header.stamp = stamp;
    marker_array.markers.push_back(box_marker);

    // ID label (TEXT_VIEW_FACING) above the box
    std::string label_text = class_id;
    if (!det.id.empty()) {
      label_text = det.id + "\n" + class_id;
    }

    geometry_msgs::msg::Point label_pos = det.bbox.center.position;
    label_pos.z += det.bbox.size.z / 2.0 + label_text_height_;

    auto label_color = lanelet_markers::makeColor(1.0f, 1.0f, 1.0f, 0.9f);
    auto label_marker = lanelet_markers::createTextMarker(
      "wo_labels", marker_id++, frame_id, label_pos, label_text, label_color,
      label_text_height_);
    label_marker.header.stamp = stamp;
    marker_array.markers.push_back(label_marker);

    // Predictions (LINE_STRIP per path)
    for (const auto & prediction : world_obj.predictions) {
      if (prediction.poses.empty()) {
        continue;
      }

      std::vector<geometry_msgs::msg::Point> pred_points;
      // Start from the object's current position
      pred_points.push_back(det.bbox.center.position);
      for (const auto & pose_stamped : prediction.poses) {
        pred_points.push_back(pose_stamped.pose.position);
      }

      auto pred_color = color;
      pred_color.a = static_cast<float>(prediction.conf * 0.8);

      auto pred_marker = lanelet_markers::createLineStripMarker(
        "wo_predictions", marker_id++, frame_id, pred_points, pred_color,
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
  rclcpp::spin(std::make_shared<world_model_markers::WorldObjectsMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
