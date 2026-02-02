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

#include "track_viz_2d/track_viz_2d.hpp"

#include <string>
#include <vector>

track_viz_2d::track_viz_2d()
: Node("track_viz_2d")
{
  initializeParams();

  // Subscribers
  dets_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    kDetectionsTopic, 10, std::bind(&track_viz_2d::detectionsCallback, this, std::placeholders::_1));
  trks_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    kTracksTopic, 10, std::bind(&track_viz_2d::tracksCallback, this, std::placeholders::_1));

  // Publishers
  annotations_pub_ = this->create_publisher<foxglove_msgs::msg::ImageAnnotations>(kAnnotationsTopic, 10);
}

void track_viz_2d::initializeParams()
{
  // Declare parameters
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "CAM_FRONT");
  color_dets_ = this->declare_parameter<std::string>("color_dets", "blue");
  color_trks_ = this->declare_parameter<std::string>("color_trks", "red");
  bbox_line_width_ = this->declare_parameter<int>("bbox_line_width", 5);

  // RGBA
  color_map_ = {
    {"red", initColor(1.0, 0.0, 0.0, 1.0)},
    {"green", initColor(0.0, 1.0, 0.0, 1.0)},
    {"blue", initColor(0.0, 0.0, 1.0, 1.0)},
    {"white", initColor(1.0, 1.0, 1.0, 1.0)},
    {"black", initColor(0.0, 0.0, 0.0, 1.0)},
  };

  default_color_ = initColor(0.0, 0.0, 1.0, 1.0);  // RGBA

  latest_dets_ = nullptr;
  latest_trks_ = nullptr;

  RCLCPP_INFO(this->get_logger(), "Parameters initialized");
}

// Initialize foxglove Color from rbga values
foxglove_msgs::msg::Color track_viz_2d::initColor(double r, double g, double b, double a)
{
  foxglove_msgs::msg::Color color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

// Initialize foxglove Point2 from x y
foxglove_msgs::msg::Point2 track_viz_2d::initPoint2(double x, double y)
{
  foxglove_msgs::msg::Point2 pt;
  pt.x = x;
  pt.y = y;
  return pt;
}

// Get color from color_map_ using key
foxglove_msgs::msg::Color track_viz_2d::colorLookup(const std::string & color)
{
  auto it = color_map_.find(color);
  if (it != color_map_.end())
    return it->second;
  else {  // Color key not in map
    RCLCPP_WARN(this->get_logger(), "Color '%s' not found, using default", color.c_str());
    return default_color_;
  }
}

// Convert a Detection2D message into a PointsAnnotation message
foxglove_msgs::msg::PointsAnnotation track_viz_2d::det2DToBox(
  const vision_msgs::msg::Detection2D & det2d, const foxglove_msgs::msg::Color & color)
{
  // Initialize bbox
  foxglove_msgs::msg::PointsAnnotation bbox;
  bbox.timestamp = this->now();
  bbox.type = bbox.LINE_LOOP;  // Closed polygon

  // Push box corners (top-left, top-right, bottom-right, bottom-left)
  bbox.points.push_back(initPoint2(
    det2d.bbox.center.position.x - det2d.bbox.size_x / 2, det2d.bbox.center.position.y - det2d.bbox.size_y / 2));
  bbox.points.push_back(initPoint2(
    det2d.bbox.center.position.x + det2d.bbox.size_x / 2, det2d.bbox.center.position.y - det2d.bbox.size_y / 2));
  bbox.points.push_back(initPoint2(
    det2d.bbox.center.position.x + det2d.bbox.size_x / 2, det2d.bbox.center.position.y + det2d.bbox.size_y / 2));
  bbox.points.push_back(initPoint2(
    det2d.bbox.center.position.x - det2d.bbox.size_x / 2, det2d.bbox.center.position.y + det2d.bbox.size_y / 2));

  // Color and width
  bbox.outline_color = color;
  bbox.thickness = bbox_line_width_;

  return bbox;
}

// Attempt box drawing
void track_viz_2d::tryAnnotation()
{
  // Check if image or detections are missing for whatever reason
  if (!latest_trks_ || !latest_dets_) {
    RCLCPP_WARN(this->get_logger(), "Missing image or tracks");
    return;
  }

  const std::vector<vision_msgs::msg::Detection2D> dets = latest_dets_->detections;
  const std::vector<vision_msgs::msg::Detection2D> trks = latest_trks_->detections;

  foxglove_msgs::msg::ImageAnnotations bboxes;

  foxglove_msgs::msg::Color det_color = colorLookup(color_dets_);
  foxglove_msgs::msg::Color trk_color = colorLookup(color_trks_);

  // Push bboxes for detections and tracks
  for (const auto & trk : trks) bboxes.points.push_back(det2DToBox(trk, trk_color));
  for (const auto & det : dets) bboxes.points.push_back(det2DToBox(det, det_color));

  annotations_pub_->publish(bboxes);
}

// Save most recent detections
void track_viz_2d::detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  latest_dets_ = msg;
}

// Image annotation triggered upon receiving tracks
void track_viz_2d::tracksCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  latest_trks_ = msg;
  tryAnnotation();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<track_viz_2d>());
  rclcpp::shutdown();
  return 0;
}
