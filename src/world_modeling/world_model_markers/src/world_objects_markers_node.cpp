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
  this->declare_parameter("label_text_height", 0.2);
  this->declare_parameter("history_line_width", 0.1);
  this->declare_parameter("prediction_line_width", 0.08);
  this->declare_parameter("lanelet_boundary_line_width", 0.05);
  this->declare_parameter("lanelet_centerline_line_width", 0.03);
  this->declare_parameter("lanelet_boundary_color_r", 0.3);
  this->declare_parameter("lanelet_boundary_color_g", 0.8);
  this->declare_parameter("lanelet_boundary_color_b", 0.3);
  this->declare_parameter("lanelet_boundary_alpha", 0.3);
  this->declare_parameter("lanelet_centerline_color_r", 0.5);
  this->declare_parameter("lanelet_centerline_color_g", 0.5);
  this->declare_parameter("lanelet_centerline_color_b", 0.5);
  this->declare_parameter("lanelet_centerline_alpha", 0.2);
  this->declare_parameter<int64_t>("entity_class_hypothesis_index", 0);

  frame_id_ = this->get_parameter("frame_id").as_string();
  box_alpha_ = this->get_parameter("box_alpha").as_double();
  label_text_height_ = this->get_parameter("label_text_height").as_double();
  history_line_width_ = this->get_parameter("history_line_width").as_double();
  prediction_line_width_ = this->get_parameter("prediction_line_width").as_double();
  lanelet_boundary_line_width_ = this->get_parameter("lanelet_boundary_line_width").as_double();
  lanelet_centerline_line_width_ = this->get_parameter("lanelet_centerline_line_width").as_double();
  lanelet_boundary_color_r_ = this->get_parameter("lanelet_boundary_color_r").as_double();
  lanelet_boundary_color_g_ = this->get_parameter("lanelet_boundary_color_g").as_double();
  lanelet_boundary_color_b_ = this->get_parameter("lanelet_boundary_color_b").as_double();
  lanelet_boundary_alpha_ = this->get_parameter("lanelet_boundary_alpha").as_double();
  lanelet_centerline_color_r_ = this->get_parameter("lanelet_centerline_color_r").as_double();
  lanelet_centerline_color_g_ = this->get_parameter("lanelet_centerline_color_g").as_double();
  lanelet_centerline_color_b_ = this->get_parameter("lanelet_centerline_color_b").as_double();
  lanelet_centerline_alpha_ = this->get_parameter("lanelet_centerline_alpha").as_double();
  hypothesis_idx_ = this->get_parameter("entity_class_hypothesis_index").as_int();

  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  subscription_ = this->create_subscription<world_model_msgs::msg::WorldObjectArray>(
    "world_objects", 10, std::bind(&WorldObjectsMarkersNode::worldObjectsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "WorldObjectsMarkersNode initialized");
}

std_msgs::msg::ColorRGBA WorldObjectsMarkersNode::getColorForClassId(const std::string & class_id) const
{
  if (class_id == "car" || class_id == "Car" || class_id == "CAR" || class_id == "vehicle" || class_id == "truck") {
    return lanelet_markers::makeColor(0.2f, 0.4f, 1.0f, static_cast<float>(box_alpha_));
  } else if (
    class_id == "human" || class_id == "Human" || class_id == "HUMAN" || class_id == "pedestrian" ||
    class_id == "Pedestrian" || class_id == "PEDESTRIAN" || class_id == "person")
  {
    return lanelet_markers::makeColor(1.0f, 1.0f, 0.0f, static_cast<float>(box_alpha_));
  } else if (class_id == "bicycle" || class_id == "Bicycle" || class_id == "BICYCLE" || class_id == "cyclist") {
    return lanelet_markers::makeColor(0.0f, 1.0f, 1.0f, static_cast<float>(box_alpha_));
  } else if (
    class_id == "motorcycle" || class_id == "Motorcycle" || class_id == "MOTORCYCLE" || class_id == "motorbike")
  {
    return lanelet_markers::makeColor(1.0f, 0.0f, 1.0f, static_cast<float>(box_alpha_));
  } else if (class_id == "traffic_light" || class_id == "TrafficLight" || class_id == "TRAFFIC_LIGHT") {
    return lanelet_markers::makeColor(1.0f, 1.0f, 1.0f, static_cast<float>(box_alpha_));
  }
  // UNKNOWN / default
  return lanelet_markers::makeColor(0.6f, 0.6f, 0.6f, static_cast<float>(box_alpha_));
}

void WorldObjectsMarkersNode::worldObjectsCallback(const world_model_msgs::msg::WorldObjectArray::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int32_t marker_id = 0;

  std::string frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;

  auto stamp = msg->header.stamp;
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    stamp = this->get_clock()->now();
  }

  // Delete all previous markers
  std::vector<std::string> namespaces = {"wo_boxes", "wo_labels", "wo_history", "wo_predictions", "wo_lanelet_ahead"};
  for (const auto & ns : namespaces) {
    auto delete_marker = lanelet_markers::createDeleteAllMarker(ns, frame_id);
    delete_marker.header.stamp = stamp;
    marker_array.markers.push_back(delete_marker);
  }

  for (const auto & world_obj : msg->objects) {
    const auto & det = world_obj.detection;

    // Get class ID from detection results using configurable hypothesis index
    std::string class_id = "unknown";
    if (!det.results.empty() && static_cast<size_t>(hypothesis_idx_) < det.results.size()) {
      class_id = det.results[hypothesis_idx_].hypothesis.class_id;
    }

    auto color = getColorForClassId(class_id);

    // Bounding box (CUBE)
    auto box_marker =
      lanelet_markers::createCubeMarker("wo_boxes", marker_id++, frame_id, det.bbox.center, det.bbox.size, color);
    box_marker.header.stamp = stamp;
    marker_array.markers.push_back(box_marker);

    // ID label (TEXT_VIEW_FACING) above the box
    std::string label_text = class_id;
    if (!det.id.empty()) {
      label_text = "ID:" + det.id + "\nCLS:" + class_id;
    }
    if (world_obj.lanelet_ahead.current_lanelet_id > 0) {
      label_text += "\nL:" + std::to_string(world_obj.lanelet_ahead.current_lanelet_id);
    }
    if (world_obj.regulatory_element.id > 0) {
      label_text += "\nRE:" + std::to_string(world_obj.regulatory_element.id);
    }
    if (world_obj.matched_way_id > 0) {
      label_text += " W:" + std::to_string(world_obj.matched_way_id);
    }

    geometry_msgs::msg::Point label_pos = det.bbox.center.position;
    label_pos.z += det.bbox.size.z / 2.0 + label_text_height_;

    auto label_color = lanelet_markers::makeColor(1.0f, 1.0f, 1.0f, 0.8f);
    auto label_marker = lanelet_markers::createTextMarker(
      "wo_labels", marker_id++, frame_id, label_pos, label_text, label_color, label_text_height_);
    label_marker.header.stamp = stamp;
    marker_array.markers.push_back(label_marker);

    // History trail (LINE_STRIP from history poses, if non-empty)
    if (world_obj.history.size() >= 2) {
      std::vector<geometry_msgs::msg::Point> history_points;
      // Current position first
      history_points.push_back(det.bbox.center.position);
      for (const auto & pose_stamped : world_obj.history) {
        history_points.push_back(pose_stamped.pose.position);
      }

      auto history_color = color;
      history_color.a = 0.4f;

      auto history_marker = lanelet_markers::createLineStripMarker(
        "wo_history", marker_id++, frame_id, history_points, history_color, history_line_width_);
      history_marker.header.stamp = stamp;
      marker_array.markers.push_back(history_marker);
    }

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
        "wo_predictions", marker_id++, frame_id, pred_points, pred_color, prediction_line_width_);
      pred_marker.header.stamp = stamp;
      marker_array.markers.push_back(pred_marker);
    }

    // Lanelet ahead visualization (boundaries/centerlines of reachable lanelets)
    if (!world_obj.lanelet_ahead.lanelets.empty()) {
      auto lanelet_color = lanelet_markers::makeColor(
        static_cast<float>(lanelet_boundary_color_r_),
        static_cast<float>(lanelet_boundary_color_g_),
        static_cast<float>(lanelet_boundary_color_b_),
        static_cast<float>(lanelet_boundary_alpha_));
      auto centerline_color = lanelet_markers::makeColor(
        static_cast<float>(lanelet_centerline_color_r_),
        static_cast<float>(lanelet_centerline_color_g_),
        static_cast<float>(lanelet_centerline_color_b_),
        static_cast<float>(lanelet_centerline_alpha_));

      for (const auto & lanelet : world_obj.lanelet_ahead.lanelets) {
        // Left boundary
        if (lanelet.left_boundary.size() >= 2) {
          auto left_marker = lanelet_markers::createLineStripMarker(
            "wo_lanelet_ahead",
            marker_id++,
            frame_id,
            lanelet.left_boundary,
            lanelet_color,
            lanelet_boundary_line_width_);
          left_marker.header.stamp = stamp;
          marker_array.markers.push_back(left_marker);
        }

        // Right boundary
        if (lanelet.right_boundary.size() >= 2) {
          auto right_marker = lanelet_markers::createLineStripMarker(
            "wo_lanelet_ahead",
            marker_id++,
            frame_id,
            lanelet.right_boundary,
            lanelet_color,
            lanelet_boundary_line_width_);
          right_marker.header.stamp = stamp;
          marker_array.markers.push_back(right_marker);
        }

        // Centerline
        if (lanelet.centerline.size() >= 2) {
          auto cl_marker = lanelet_markers::createLineStripMarker(
            "wo_lanelet_ahead",
            marker_id++,
            frame_id,
            lanelet.centerline,
            centerline_color,
            lanelet_centerline_line_width_);
          cl_marker.header.stamp = stamp;
          marker_array.markers.push_back(cl_marker);
        }
      }
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
