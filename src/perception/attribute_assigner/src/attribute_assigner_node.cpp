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

#include "attribute_assigner/attribute_assigner_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/image_marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "attribute_assigner/car_behavior_classifier.hpp"
#include "attribute_assigner/traffic_light_classifier.hpp"

namespace wato::perception::attribute_assigner
{

AttributeAssignerNode::AttributeAssignerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("attribute_assigner_node", options)
, subscriber_qos_(10)
, publisher_qos_(10)
, last_stats_log_time_(std::chrono::steady_clock::now())
{
  RCLCPP_INFO(this->get_logger(), "Attribute Assigner ROS 2 lifecycle node created");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", this->get_current_state().label().c_str());
}

void AttributeAssignerNode::declareParameters()
{
  // Minimum detection confidence (core-level)
  this->declare_parameter<double>("min_detection_confidence", 0.3);
  const double min_confidence = this->get_parameter("min_detection_confidence").as_double();

  core_ = std::make_unique<AttributeAssignerCore>(min_confidence);

  // --- Traffic light classifier params ---
  this->declare_parameter<std::vector<std::string>>(
    "traffic_light_class_ids", std::vector<std::string>{"traffic light"});
  this->declare_parameter<double>("traffic_light_min_saturation", 60.0);
  this->declare_parameter<double>("traffic_light_min_value", 80.0);
  this->declare_parameter<double>("traffic_light_red_hue_lo", 10.0);
  this->declare_parameter<double>("traffic_light_red_hue_hi", 170.0);
  this->declare_parameter<double>("traffic_light_yellow_hue_hi", 35.0);
  this->declare_parameter<double>("traffic_light_green_hue_hi", 85.0);
  this->declare_parameter<double>("traffic_light_strip_margin", 0.20);

  this->declare_parameter<double>("traffic_light_assumed_depth", 30.0);

  TrafficLightClassifier::Params tl_params;
  tl_params.class_ids = this->get_parameter("traffic_light_class_ids").as_string_array();
  tl_params.min_saturation = this->get_parameter("traffic_light_min_saturation").as_double();
  tl_params.min_value = this->get_parameter("traffic_light_min_value").as_double();
  tl_params.red_hue_lo = this->get_parameter("traffic_light_red_hue_lo").as_double();
  tl_params.red_hue_hi = this->get_parameter("traffic_light_red_hue_hi").as_double();
  tl_params.yellow_hue_hi = this->get_parameter("traffic_light_yellow_hue_hi").as_double();
  tl_params.green_hue_hi = this->get_parameter("traffic_light_green_hue_hi").as_double();
  tl_params.strip_margin = this->get_parameter("traffic_light_strip_margin").as_double();
  tl_params.assumed_depth = this->get_parameter("traffic_light_assumed_depth").as_double();

  core_->addClassifier(std::make_unique<TrafficLightClassifier>(tl_params));

  // --- Car behavior classifier params ---
  this->declare_parameter<std::vector<std::string>>("car_class_ids", std::vector<std::string>{"car"});
  this->declare_parameter<double>("car_brake_min_brightness", 90.0);
  this->declare_parameter<double>("car_brake_min_saturation", 40.0);
  this->declare_parameter<double>("car_red_hue_lo", 10.0);
  this->declare_parameter<double>("car_red_hue_hi", 170.0);
  this->declare_parameter<double>("car_amber_hue_lo", 12.0);
  this->declare_parameter<double>("car_amber_hue_hi", 35.0);
  this->declare_parameter<double>("car_amber_min_saturation", 80.0);
  this->declare_parameter<double>("car_amber_min_value", 100.0);

  // Vehicle subtype IDs and 3D dimensions (owned by car classifier)
  this->declare_parameter<std::vector<std::string>>("truck_class_ids", std::vector<std::string>{"truck"});
  this->declare_parameter<std::vector<std::string>>("bus_class_ids", std::vector<std::string>{"bus"});
  this->declare_parameter<double>("car_real_width", 1.8);
  this->declare_parameter<double>("car_real_height", 1.5);
  this->declare_parameter<double>("car_real_length", 4.5);
  this->declare_parameter<double>("truck_real_width", 2.5);
  this->declare_parameter<double>("truck_real_height", 3.5);
  this->declare_parameter<double>("truck_real_length", 8.0);
  this->declare_parameter<double>("bus_real_width", 2.5);
  this->declare_parameter<double>("bus_real_height", 3.2);
  this->declare_parameter<double>("bus_real_length", 12.0);
  this->declare_parameter<double>("car_assumed_depth", 20.0);

  CarBehaviorClassifier::Params car_params;
  car_params.class_ids = this->get_parameter("car_class_ids").as_string_array();
  car_params.brake_min_brightness = this->get_parameter("car_brake_min_brightness").as_double();
  car_params.brake_min_saturation = this->get_parameter("car_brake_min_saturation").as_double();
  car_params.red_hue_lo = this->get_parameter("car_red_hue_lo").as_double();
  car_params.red_hue_hi = this->get_parameter("car_red_hue_hi").as_double();
  car_params.amber_hue_lo = this->get_parameter("car_amber_hue_lo").as_double();
  car_params.amber_hue_hi = this->get_parameter("car_amber_hue_hi").as_double();
  car_params.amber_min_saturation = this->get_parameter("car_amber_min_saturation").as_double();
  car_params.amber_min_value = this->get_parameter("car_amber_min_value").as_double();
  car_params.truck_class_ids = this->get_parameter("truck_class_ids").as_string_array();
  car_params.bus_class_ids = this->get_parameter("bus_class_ids").as_string_array();
  car_params.car_width = this->get_parameter("car_real_width").as_double();
  car_params.car_height = this->get_parameter("car_real_height").as_double();
  car_params.car_length = this->get_parameter("car_real_length").as_double();
  car_params.truck_width = this->get_parameter("truck_real_width").as_double();
  car_params.truck_height = this->get_parameter("truck_real_height").as_double();
  car_params.truck_length = this->get_parameter("truck_real_length").as_double();
  car_params.bus_width = this->get_parameter("bus_real_width").as_double();
  car_params.bus_height = this->get_parameter("bus_real_height").as_double();
  car_params.bus_length = this->get_parameter("bus_real_length").as_double();
  car_params.assumed_depth = this->get_parameter("car_assumed_depth").as_double();

  core_->addClassifier(std::make_unique<CarBehaviorClassifier>(car_params));
}

void AttributeAssignerNode::syncedCallback(
  const deep_msgs::msg::MultiImageCompressed::ConstSharedPtr & multi_image_msg,
  const deep_msgs::msg::MultiDetection2DArray::ConstSharedPtr & detections_msg)
{
  multi_image_msg_count_++;
  detections_msg_count_++;
  synced_msg_count_++;

  if (!core_) {
    RCLCPP_WARN(this->get_logger(), "[SYNC] Core not initialized; skipping");
    return;
  }

  if (!multi_image_msg || multi_image_msg->images.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[SYNC] Received empty MultiImage; passing %zu camera detection arrays through",
      detections_msg->camera_detections.size());
    if (detections_pub_ && detections_pub_->is_activated()) {
      detections_pub_->publish(*detections_msg);
    }
    return;
  }

  const auto start = std::chrono::steady_clock::now();

  // Build frame_id -> MultiImage index lookup for O(1) access
  std::unordered_map<std::string, size_t> frame_id_to_index;
  for (size_t i = 0; i < multi_image_msg->images.size(); ++i) {
    const auto & frame_id = multi_image_msg->images[i].header.frame_id;
    frame_id_to_index[frame_id] = i;
  }

  // First pass: collect unique frame_ids that have classifiable detections
  std::unordered_set<std::string> frames_to_decompress;

  size_t total_dets = 0;
  size_t classifiable_dets = 0;

  for (const auto & camera_det : detections_msg->camera_detections) {
    const std::string & camera_frame_id = camera_det.header.frame_id;
    total_dets += camera_det.detections.size();
    for (const auto & det : camera_det.detections) {
      const auto * classifier = core_->findClassifier(det);
      if (classifier == nullptr) {
        continue;
      }
      classifiable_dets++;
      const double score = core_->getBestScore(det);
      if (score < core_->getMinDetectionConfidence()) {
        continue;
      }
      // Use the camera detection array's frame_id to identify which camera this came from
      if (!camera_frame_id.empty() && frame_id_to_index.count(camera_frame_id) > 0) {
        frames_to_decompress.insert(camera_frame_id);
        break;  // Already marked this camera, no need to check more detections
      }
    }
  }

  // If no frames to decompress, pass through
  if (frames_to_decompress.empty()) {
    if (detections_pub_ && detections_pub_->is_activated()) {
      detections_pub_->publish(*detections_msg);
    }
    return;
  }

  // Decompress only the needed images (once per frame)
  std::unordered_map<std::string, cv::Mat> decompressed_images;
  for (const auto & frame_id : frames_to_decompress) {
    const size_t idx = frame_id_to_index[frame_id];
    cv::Mat image = decompressImage(multi_image_msg->images[idx]);
    if (!image.empty()) {
      decompressed_images[frame_id] = std::move(image);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Failed to decompress image for frame_id '%s'", frame_id.c_str());
    }
  }

  // Process detections per camera
  deep_msgs::msg::MultiDetection2DArray enriched;
  enriched.header = detections_msg->header;
  enriched.camera_detections.reserve(detections_msg->camera_detections.size());

  size_t enriched_count = 0;

  for (const auto & camera_det : detections_msg->camera_detections) {
    vision_msgs::msg::Detection2DArray enriched_camera;
    enriched_camera.header = camera_det.header;
    enriched_camera.detections.reserve(camera_det.detections.size());

    const std::string & frame_id = camera_det.header.frame_id;
    auto img_it = decompressed_images.find(frame_id);

    for (const auto & det : camera_det.detections) {
      vision_msgs::msg::Detection2D enriched_det = det;

      // Find a classifier that handles this detection
      const auto * classifier = core_->findClassifier(det);
      if (classifier == nullptr) {
        enriched_camera.detections.push_back(enriched_det);
        continue;
      }

      const double score = core_->getBestScore(det);
      if (score < core_->getMinDetectionConfidence()) {
        enriched_camera.detections.push_back(enriched_det);
        continue;
      }

      if (img_it == decompressed_images.end() || img_it->second.empty()) {
        enriched_camera.detections.push_back(enriched_det);
        continue;
      }

      cv::Mat crop = core_->cropToBbox(img_it->second, det);
      if (crop.empty()) {
        enriched_camera.detections.push_back(enriched_det);
        continue;
      }

      classifier->classify(crop, enriched_det);
      enriched_count++;

      enriched_camera.detections.push_back(enriched_det);
    }

    enriched.camera_detections.push_back(enriched_camera);
  }

  const auto end = std::chrono::steady_clock::now();
  const double time_taken = std::chrono::duration<double, std::milli>(end - start).count();
  updateStatistics(time_taken);

  if (detections_pub_ && detections_pub_->is_activated()) {
    detections_pub_->publish(enriched);
  }

  // Publish image markers for visualization in Foxglove
  if (enable_image_markers_ && image_markers_pub_ && image_markers_pub_->is_activated() && !decompressed_images.empty())
  {
    // Use the first decompressed image dimensions for validation
    auto first_img_it = decompressed_images.begin();

    if (!first_img_it->second.empty()) {
      auto markers = createDetectionMarkers(enriched, decompressed_images, detections_msg->header.stamp);

      // Publish each marker individually
      for (const auto & marker : markers) {
        image_markers_pub_->publish(marker);
      }
    }
  }
  // Create and publish 3D detections (traffic lights + cars)
  if (detections_3d_pub_ && detections_3d_pub_->is_activated()) {
    MultiCameraInfoMsg::ConstSharedPtr cached_camera_info;
    {
      std::lock_guard<std::mutex> lock(sync_mutex_);
      cached_camera_info = cached_multi_camera_info_;
    }

    if (cached_camera_info && !cached_camera_info->camera_infos.empty()) {
      // Build frame_id -> CameraInfo lookup
      std::unordered_map<std::string, sensor_msgs::msg::CameraInfo> camera_infos;
      for (size_t i = 0; i < multi_image_msg->images.size() && i < cached_camera_info->camera_infos.size(); ++i) {
        const std::string & frame_id = multi_image_msg->images[i].header.frame_id;
        camera_infos[frame_id] = cached_camera_info->camera_infos[i];
      }

      auto detections_3d =
        create3DDetections(enriched, camera_infos, detections_msg->camera_detections[0].header.stamp);
      detections_3d_pub_->publish(detections_3d);

      if (enable_3d_markers_ && detections_3d_markers_pub_ && detections_3d_markers_pub_->is_activated()) {
        auto markers = create3DMarkers(detections_3d);
        detections_3d_markers_pub_->publish(markers);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "3D detections skipped: no cached MultiCameraInfo (topic: '%s')",
        multi_camera_info_topic_.c_str());
    }
  }

  updateDiagnostics(detections_msg->header.stamp);
}

void AttributeAssignerNode::multiCameraInfoCallback(const deep_msgs::msg::MultiCameraInfo::ConstSharedPtr & msg)
{
  if (msg->camera_infos.empty()) {
    return;  // Skip empty messages
  }

  // Cache the latest camera info message
  std::lock_guard<std::mutex> lock(sync_mutex_);
  cached_multi_camera_info_ = msg;
}

cv::Mat AttributeAssignerNode::decompressImage(const sensor_msgs::msg::CompressedImage & compressed_img) const
{
  try {
    cv::Mat image = cv::imdecode(cv::Mat(compressed_img.data), cv::IMREAD_COLOR);
    return image;  // Returns empty Mat on failure
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "OpenCV exception during decompression: %s", e.what());
    return cv::Mat();
  }
}

std::vector<visualization_msgs::msg::ImageMarker> AttributeAssignerNode::createDetectionMarkers(
  const deep_msgs::msg::MultiDetection2DArray & detections,
  const std::unordered_map<std::string, cv::Mat> & decompressed_images,
  const builtin_interfaces::msg::Time & stamp) const
{
  std::vector<visualization_msgs::msg::ImageMarker> markers;

  if (!core_) {
    return markers;
  }

  int marker_id = 0;

  for (const auto & camera_det : detections.camera_detections) {
    const std::string & frame_id = camera_det.header.frame_id;
    auto img_it = decompressed_images.find(frame_id);
    if (img_it == decompressed_images.end() || img_it->second.empty()) {
      continue;  // Skip if image not available for this camera
    }

    const cv::Mat & image = img_it->second;
    const int image_width = image.cols;
    const int image_height = image.rows;

    for (const auto & det : camera_det.detections) {
      const auto * classifier = core_->findClassifier(det);
      if (classifier == nullptr) {
        continue;
      }

      if (core_->getBestScore(det) < core_->getMinDetectionConfidence()) {
        continue;
      }

      // Get bounding box coordinates
      const double cx = det.bbox.center.position.x;
      const double cy = det.bbox.center.position.y;
      const double w = det.bbox.size_x;
      const double h = det.bbox.size_y;

      if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h)) {
        continue;
      }
      if (w <= 0 || h <= 0) {
        continue;
      }

      const int x1 = static_cast<int>(std::round(cx - w / 2.0));
      const int y1 = static_cast<int>(std::round(cy - h / 2.0));
      const int x2 = static_cast<int>(std::round(cx + w / 2.0));
      const int y2 = static_cast<int>(std::round(cy + h / 2.0));

      const int x1_c = std::max(0, std::min(x1, image_width - 1));
      const int y1_c = std::max(0, std::min(y1, image_height - 1));
      const int x2_c = std::max(0, std::min(x2, image_width));
      const int y2_c = std::max(0, std::min(y2, image_height));

      if (x1_c >= x2_c || y1_c >= y2_c) {
        continue;
      }

      // Get color from the classifier based on enriched attributes
      MarkerColor color = classifier->getMarkerColor(det);

      // === BOUNDING BOX (LINE_STRIP for rectangle) ===
      {
        visualization_msgs::msg::ImageMarker bbox_marker;
        bbox_marker.header.frame_id = frame_id;
        bbox_marker.header.stamp = stamp;
        bbox_marker.ns = "bounding_boxes";
        bbox_marker.id = marker_id++;
        bbox_marker.type = visualization_msgs::msg::ImageMarker::LINE_STRIP;
        bbox_marker.action = visualization_msgs::msg::ImageMarker::ADD;
        bbox_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        bbox_marker.outline_color.r = color.r;
        bbox_marker.outline_color.g = color.g;
        bbox_marker.outline_color.b = color.b;
        bbox_marker.outline_color.a = 1.0;
        bbox_marker.scale = 3.0;

        geometry_msgs::msg::Point p1, p2, p3, p4, p5;
        p1.x = x1_c;
        p1.y = y1_c;
        p2.x = x2_c;
        p2.y = y1_c;
        p3.x = x2_c;
        p3.y = y2_c;
        p4.x = x1_c;
        p4.y = y2_c;
        p5.x = x1_c;
        p5.y = y1_c;

        bbox_marker.points = {p1, p2, p3, p4, p5};
        markers.push_back(bbox_marker);
      }
    }  // end per-detection loop
  }  // end per-camera loop

  return markers;
}

vision_msgs::msg::Detection3DArray AttributeAssignerNode::create3DDetections(
  const deep_msgs::msg::MultiDetection2DArray & detections,
  const std::unordered_map<std::string, sensor_msgs::msg::CameraInfo> & camera_infos,
  const builtin_interfaces::msg::Time & stamp) const
{
  vision_msgs::msg::Detection3DArray detections_3d;
  detections_3d.header.stamp = stamp;
  detections_3d.header.frame_id = target_frame_;

  if (!core_ || !tf_buffer_) {
    return detections_3d;
  }

  for (const auto & camera_det : detections.camera_detections) {
    const std::string & frame_id = camera_det.header.frame_id;
    auto camera_info_it = camera_infos.find(frame_id);
    if (camera_info_it == camera_infos.end()) {
      continue;
    }

    const sensor_msgs::msg::CameraInfo & cam_info = camera_info_it->second;
    if (cam_info.k.size() != 9) {
      continue;
    }

    CameraIntrinsics intrinsics;
    intrinsics.fx = cam_info.k[0];
    intrinsics.fy = cam_info.k[4];
    intrinsics.cx = cam_info.k[2];
    intrinsics.cy = cam_info.k[5];

    for (const auto & det : camera_det.detections) {
      const auto * classifier = core_->findClassifier(det);
      if (classifier == nullptr) {
        continue;
      }

      if (core_->getBestScore(det) < core_->getMinDetectionConfidence()) {
        continue;
      }

      // Get 3D box params from the classifier (handles subtype resolution)
      BoxParams3D box_params = classifier->get3DBoxParams(det);

      // Pure math: project 2D detection to 3D in camera frame
      Projection3D proj = AttributeAssignerCore::projectTo3D(intrinsics, det, box_params);

      // ROS: transform from camera frame to target frame
      geometry_msgs::msg::PointStamped point_camera;
      point_camera.header.frame_id = frame_id;
      point_camera.header.stamp = stamp;
      point_camera.point.x = proj.x;
      point_camera.point.y = proj.y;
      point_camera.point.z = proj.z;

      geometry_msgs::msg::PointStamped point_target;
      try {
        auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(100));
        point_target = tf_buffer_->transform(point_camera, target_frame_, timeout);
      } catch (const tf2::TransformException &) {
        continue;
      }

      // Build Detection3D message
      vision_msgs::msg::Detection3D det_3d;
      det_3d.header.frame_id = target_frame_;
      det_3d.header.stamp = stamp;
      det_3d.results = det.results;

      det_3d.bbox.center.position.x = point_target.point.x;
      det_3d.bbox.center.position.y = point_target.point.y;
      det_3d.bbox.center.position.z = point_target.point.z;

      const double yaw = std::atan2(point_target.point.y, point_target.point.x);
      det_3d.bbox.center.orientation.w = std::cos(yaw / 2.0);
      det_3d.bbox.center.orientation.x = 0.0;
      det_3d.bbox.center.orientation.y = 0.0;
      det_3d.bbox.center.orientation.z = std::sin(yaw / 2.0);

      det_3d.bbox.size.x = proj.size_x;
      det_3d.bbox.size.y = proj.size_y;
      det_3d.bbox.size.z = proj.size_z;

      detections_3d.detections.push_back(det_3d);
    }
  }

  return detections_3d;
}

visualization_msgs::msg::MarkerArray AttributeAssignerNode::create3DMarkers(
  const vision_msgs::msg::Detection3DArray & detections_3d) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Clear previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_marker.header = detections_3d.header;
  delete_marker.ns = "detections_3d";
  marker_array.markers.push_back(delete_marker);

  for (size_t i = 0; i < detections_3d.detections.size(); ++i) {
    const auto & det = detections_3d.detections[i];

    visualization_msgs::msg::Marker marker;
    marker.header = detections_3d.header;
    marker.ns = "detections_3d";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = det.bbox.center.position;
    marker.pose.orientation = det.bbox.center.orientation;

    marker.scale.x = det.bbox.size.x;
    marker.scale.y = det.bbox.size.y;
    marker.scale.z = det.bbox.size.z;

    // Build a Detection2D with the same results to query the classifier
    vision_msgs::msg::Detection2D det_2d;
    det_2d.results = det.results;

    // Get color from the matching classifier
    MarkerColor color{1.0f, 1.0f, 1.0f, 0.5f};  // default white
    if (core_) {
      const auto * classifier = core_->findClassifier(det_2d);
      if (classifier != nullptr) {
        color = classifier->getMarkerColor(det_2d);
      }
    }

    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;

    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

void AttributeAssignerNode::updateStatistics(double time_taken)
{
  total_processed_++;
  double current_total = total_processing_time_ms_.load();
  while (!total_processing_time_ms_.compare_exchange_weak(current_total, current_total + time_taken)) {
  }
  last_processing_time_ms_ = time_taken;

  const auto now = std::chrono::steady_clock::now();
  if (now - last_stats_log_time_ >= kStatsLogInterval) {
    logStatistics();
    last_stats_log_time_ = now;
  }
}

void AttributeAssignerNode::updateDiagnostics(const std_msgs::msg::Header::_stamp_type & timestamp)
{
  if (pub_diagnostic_) {
    pub_diagnostic_->tick(timestamp);
  }

  if (diagnostic_updater_) {
    diagnostic_updater_->force_update();
  }
}

void AttributeAssignerNode::logStatistics() const
{
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;

  RCLCPP_INFO(
    this->get_logger(), "Statistics: %lu arrays processed, average processing time: %.3f ms", processed, avg_time);
}

rclcpp::QoS AttributeAssignerNode::createSubscriberQoS(const std::string & reliability, int depth)
{
  RCLCPP_INFO(this->get_logger(), "Creating subscriber QoS: reliability='%s', depth=%d", reliability.c_str(), depth);

  rclcpp::QoS qos(depth);
  if (reliability == "reliable") {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  } else if (reliability == "best_effort") {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown reliability policy '%s', defaulting to best_effort. Valid values: 'reliable', 'best_effort'",
      reliability.c_str());
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  }
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  return qos;
}

rclcpp::QoS AttributeAssignerNode::createPublisherQoS(
  const std::string & reliability, const std::string & durability, int depth)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Creating publisher QoS: reliability='%s', durability='%s', depth=%d",
    reliability.c_str(),
    durability.c_str(),
    depth);

  rclcpp::QoS qos(depth);
  if (reliability == "reliable") {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  } else if (reliability == "best_effort") {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown reliability policy '%s', defaulting to reliable. Valid values: 'reliable', 'best_effort'",
      reliability.c_str());
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  }

  if (durability == "transient_local") {
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  } else if (durability == "volatile") {
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown durability policy '%s', defaulting to transient_local. Valid values: 'transient_local', 'volatile'",
      durability.c_str());
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
  return qos;
}

void AttributeAssignerNode::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;
  const double last_time = last_processing_time_ms_.load();

  if (processed == 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No detection arrays processed yet");
  } else if (avg_time > 50.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "High processing latency");
  } else if (last_time > 100.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Recent high processing latency");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Operating normally");
  }

  stat.add("Arrays Processed", processed);
  stat.add("Average Processing Time (ms)", avg_time);
  stat.add("Last Processing Time (ms)", last_time);
  if (core_) {
    stat.add("Total Detections Processed", core_->getProcessedCount());
  }
}

// ---------------------------------------------------------------------------
// Lifecycle callbacks
// ---------------------------------------------------------------------------

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_configure(
  const rclcpp_lifecycle::State & /* prev_state */)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Attribute Assigner node");

  try {
    declareParameters();

    // Log configured classifiers
    RCLCPP_INFO(this->get_logger(), "Configuration summary:");
    RCLCPP_INFO(this->get_logger(), "  - Registered classifiers: %zu", core_->getClassifiers().size());
    RCLCPP_INFO(this->get_logger(), "  - Min detection confidence: %.2f", core_->getMinDetectionConfidence());

    // Declare topic name parameters (for message_filters - remapping doesn't work automatically)
    this->declare_parameter<std::string>("input_multi_image_topic", std::string(kMultiImageTopic));
    this->declare_parameter<std::string>("input_multi_camera_info_topic", std::string(kMultiCameraInfoTopic));
    this->declare_parameter<std::string>("input_detections_topic", std::string(kInputTopic));
    this->declare_parameter<std::string>("output_detections_topic", std::string(kOutputTopic));

    multi_image_topic_ = this->get_parameter("input_multi_image_topic").as_string();
    multi_camera_info_topic_ = this->get_parameter("input_multi_camera_info_topic").as_string();
    input_detections_topic_ = this->get_parameter("input_detections_topic").as_string();
    output_detections_topic_ = this->get_parameter("output_detections_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Topic names configured:");
    RCLCPP_INFO(this->get_logger(), "  - Input multi-image: '%s'", multi_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Input multi-camera-info: '%s'", multi_camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Input detections: '%s'", input_detections_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Output detections: '%s'", output_detections_topic_.c_str());

    // Visualization toggles
    this->declare_parameter<bool>("enable_image_markers", true);
    this->declare_parameter<bool>("enable_3d_markers", true);
    enable_image_markers_ = this->get_parameter("enable_image_markers").as_bool();
    enable_3d_markers_ = this->get_parameter("enable_3d_markers").as_bool();

    // 3D detection: only target_frame is node-level (sizing is in classifiers)
    this->declare_parameter<std::string>("target_frame", "base_link");
    target_frame_ = this->get_parameter("target_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "3D detection settings:");
    RCLCPP_INFO(this->get_logger(), "  - Target frame: '%s'", target_frame_.c_str());

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare and configure QoS parameters
    this->declare_parameter<std::string>("qos_subscriber_reliability", "best_effort");
    this->declare_parameter<int>("qos_subscriber_depth", 10);
    this->declare_parameter<std::string>("qos_publisher_reliability", "reliable");
    this->declare_parameter<std::string>("qos_publisher_durability", "transient_local");
    this->declare_parameter<int>("qos_publisher_depth", 10);

    const std::string subscriber_reliability = this->get_parameter("qos_subscriber_reliability").as_string();
    const int subscriber_depth = this->get_parameter("qos_subscriber_depth").as_int();
    subscriber_qos_ = createSubscriberQoS(subscriber_reliability, subscriber_depth);

    const std::string publisher_reliability = this->get_parameter("qos_publisher_reliability").as_string();
    const std::string publisher_durability = this->get_parameter("qos_publisher_durability").as_string();
    const int publisher_depth = this->get_parameter("qos_publisher_depth").as_int();
    publisher_qos_ = createPublisherQoS(publisher_reliability, publisher_durability, publisher_depth);

    this->declare_parameter<int>("sync_queue_size", 10);
    this->declare_parameter<double>("sync_max_time_diff_ms", 200.0);

    sync_queue_size_ = this->get_parameter("sync_queue_size").as_int();
    sync_max_time_diff_ms_ = this->get_parameter("sync_max_time_diff_ms").as_double();
    sync_max_time_diff_sec_ = sync_max_time_diff_ms_ / 1000.0;  // Convert to seconds

    RCLCPP_INFO(this->get_logger(), "Message synchronization settings:");
    RCLCPP_INFO(this->get_logger(), "  - Queue size: %d", sync_queue_size_);
    RCLCPP_INFO(
      this->get_logger(),
      "  - Max time difference: %.1f ms (%.3f sec)",
      sync_max_time_diff_ms_,
      sync_max_time_diff_sec_);
    RCLCPP_INFO(this->get_logger(), "  - Synchronization method: ApproximateTime (MultiImage + Detections)");

    // Initialize diagnostics
    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("attribute_assigner");
    diagnostic_updater_->add("Attribute Assigner Status", this, &AttributeAssignerNode::diagnosticCallback);

    RCLCPP_INFO(this->get_logger(), "Node configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Configuration failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_activate(
  const rclcpp_lifecycle::State & prev_state)
{
  RCLCPP_INFO(this->get_logger(), "Activating Attribute Assigner node");
  RCLCPP_INFO(this->get_logger(), "Previous state: %s", prev_state.label().c_str());
  RCLCPP_INFO(this->get_logger(), "=============================================");

  try {
    // Create message_filters subscribers for ApproximateTime sync
    RCLCPP_INFO(this->get_logger(), "Creating subscribers:");
    RCLCPP_INFO(this->get_logger(), "  - MultiImage topic: '%s'", multi_image_topic_.c_str());
    multi_image_sub_ =
      std::make_shared<ImageSub>(this->shared_from_this(), multi_image_topic_, subscriber_qos_.get_rmw_qos_profile());

    RCLCPP_INFO(this->get_logger(), "  - Detections topic: '%s'", input_detections_topic_.c_str());
    detections_sub_ = std::make_shared<DetSub>(
      this->shared_from_this(), input_detections_topic_, subscriber_qos_.get_rmw_qos_profile());

    // ApproximateTime synchronizer
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(sync_queue_size_), *multi_image_sub_, *detections_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_max_time_diff_sec_));
    sync_->registerCallback(
      std::bind(&AttributeAssignerNode::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));

    // CameraInfo is cached separately (changes infrequently)
    RCLCPP_INFO(this->get_logger(), "  - MultiCameraInfo topic: '%s'", multi_camera_info_topic_.c_str());
    multi_camera_info_sub_ = this->create_subscription<MultiCameraInfoMsg>(
      multi_camera_info_topic_,
      subscriber_qos_,
      std::bind(&AttributeAssignerNode::multiCameraInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "ApproximateTime sync configured (queue=%d, max_interval=%.3fs)",
      sync_queue_size_,
      sync_max_time_diff_sec_);

    RCLCPP_INFO(this->get_logger(), "Creating publishers:");
    RCLCPP_INFO(this->get_logger(), "  - Output topic: '%s'", output_detections_topic_.c_str());
    detections_pub_ =
      this->create_publisher<deep_msgs::msg::MultiDetection2DArray>(output_detections_topic_, publisher_qos_);

    RCLCPP_INFO(this->get_logger(), "  - Image markers topic: '%s'", kImageMarkersTopic);
    image_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::ImageMarker>(kImageMarkersTopic, publisher_qos_);

    RCLCPP_INFO(this->get_logger(), "  - Traffic lights 3D topic: '%s'", kDetections3DTopic);
    detections_3d_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(kDetections3DTopic, publisher_qos_);

    RCLCPP_INFO(this->get_logger(), "  - Traffic lights 3D markers topic: '%s'", kDetections3DMarkersTopic);
    detections_3d_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(kDetections3DMarkersTopic, publisher_qos_);

    if (detections_pub_) {
      detections_pub_->on_activate();
    }

    if (image_markers_pub_) {
      image_markers_pub_->on_activate();
    }

    if (detections_3d_pub_) {
      detections_3d_pub_->on_activate();
    }

    if (detections_3d_markers_pub_) {
      detections_3d_markers_pub_->on_activate();
    }

    if (diagnostic_updater_ && detections_pub_) {
      pub_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
        output_detections_topic_,
        *diagnostic_updater_,
        diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10),
        diagnostic_updater::TimeStampStatusParam());
      RCLCPP_INFO(this->get_logger(), "Topic diagnostics ready");
    }

    RCLCPP_INFO(this->get_logger(), "=============================================");
    RCLCPP_INFO(this->get_logger(), "Node activated successfully!");
    RCLCPP_INFO(this->get_logger(), "Subscribed to:");
    RCLCPP_INFO(this->get_logger(), "  - MultiImage: '%s'", multi_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Detections: '%s'", input_detections_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to:");
    RCLCPP_INFO(this->get_logger(), "  - Enriched detections: '%s'", detections_pub_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "  - MultiCameraInfo (cached): '%s'", multi_camera_info_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "[SYNC] ApproximateTime sync: MultiImage + Detections (queue=%d, max=%.3fs)",
      sync_queue_size_,
      sync_max_time_diff_sec_);
    RCLCPP_INFO(this->get_logger(), "=============================================");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Activation failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_deactivate(
  const rclcpp_lifecycle::State & /* prev_state */)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Attribute Assigner node");

  if (detections_pub_) {
    detections_pub_->on_deactivate();
  }

  if (image_markers_pub_) {
    image_markers_pub_->on_deactivate();
  }

  if (detections_3d_pub_) {
    detections_3d_pub_->on_deactivate();
  }

  if (detections_3d_markers_pub_) {
    detections_3d_markers_pub_->on_deactivate();
  }

  sync_.reset();
  multi_image_sub_.reset();
  detections_sub_.reset();
  multi_camera_info_sub_.reset();

  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    cached_multi_camera_info_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "=============================================");
  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_cleanup(
  const rclcpp_lifecycle::State & /* prev_state */)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Attribute Assigner node");

  sync_.reset();
  multi_image_sub_.reset();
  detections_sub_.reset();
  multi_camera_info_sub_.reset();

  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    cached_multi_camera_info_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Cleaning up publishers...");
  detections_pub_.reset();
  image_markers_pub_.reset();
  detections_3d_pub_.reset();
  detections_3d_markers_pub_.reset();
  pub_diagnostic_.reset();
  diagnostic_updater_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  core_.reset();

  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "=============================================");
  RCLCPP_INFO(this->get_logger(), "Shutting down Attribute Assigner node");
  RCLCPP_INFO(this->get_logger(), "Previous state: %s", previous_state.label().c_str());
  RCLCPP_INFO(this->get_logger(), "=============================================");

  // Log final statistics before shutdown
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;

  RCLCPP_INFO(this->get_logger(), "Final statistics:");
  RCLCPP_INFO(this->get_logger(), "  - Total arrays processed: %lu", processed);
  RCLCPP_INFO(this->get_logger(), "  - Total processing time: %.3f ms", total_time);
  RCLCPP_INFO(this->get_logger(), "  - Average processing time: %.3f ms", avg_time);
  RCLCPP_INFO(this->get_logger(), "Message statistics:");
  RCLCPP_INFO(this->get_logger(), "  - MultiImage messages received: %lu", multi_image_msg_count_.load());
  RCLCPP_INFO(this->get_logger(), "  - Detection messages received: %lu", detections_msg_count_.load());
  RCLCPP_INFO(this->get_logger(), "  - Synchronized callbacks: %lu", synced_msg_count_.load());

  RCLCPP_INFO(this->get_logger(), "Resetting all resources...");
  sync_.reset();
  multi_image_sub_.reset();
  detections_sub_.reset();
  multi_camera_info_sub_.reset();
  detections_pub_.reset();
  image_markers_pub_.reset();
  detections_3d_pub_.reset();
  detections_3d_markers_pub_.reset();
  cached_multi_camera_info_.reset();
  core_.reset();
  pub_diagnostic_.reset();
  diagnostic_updater_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  total_processed_ = 0;
  total_processing_time_ms_ = 0.0;
  last_processing_time_ms_ = 0.0;

  RCLCPP_INFO(this->get_logger(), "Node shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace wato::perception::attribute_assigner

RCLCPP_COMPONENTS_REGISTER_NODE(wato::perception::attribute_assigner::AttributeAssignerNode)
