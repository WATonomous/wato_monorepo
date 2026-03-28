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

#include "tracking/tracking.hpp"

#include <algorithm>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// static logger for static logging
rclcpp::Logger TrackingNode::static_logger_ = rclcpp::get_logger("tracking_stc");

// class maps
std::unordered_map<std::string, int> TrackingNode::class_map_ = {
  {"car", 0}, {"truck", 1}, {"bicycle", 2}, {"person", 3}, {"bus", 4}, {"vehicle", 5}, {"traffic light", 6}
  // etc...
};
std::unordered_map<int, std::string> TrackingNode::reverse_class_map_ = [] {
  std::unordered_map<int, std::string> m;
  for (const auto & [k, v] : TrackingNode::class_map_) m[v] = k;
  return m;
}();

TrackingNode::TrackingNode(const rclcpp::NodeOptions & options)
: LifecycleNode("tracking", options)
{
  RCLCPP_INFO(this->get_logger(), "New tracking node created");
}

TrackingNode::CallbackReturn TrackingNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring tracking node...");

  // Parameters
  initializeParams();

  // TF stuff
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriber
  dets_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    kDetectionsTopic, 10, std::bind(&TrackingNode::detectionsCallback, this, std::placeholders::_1));
  RCLCPP_DEBUG(this->get_logger(), "Subscriber initialized, subscribed to %s", dets_sub_->get_topic_name());

  // Lifecycle Publishers
  tracked_dets_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(kTracksTopic, 10);
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kMarkersTopic, 10);
  RCLCPP_DEBUG(this->get_logger(), "Publishers initialized, publishing to %s", tracked_dets_pub_->get_topic_name());

  // ByteTrack tracker
  tracker_ = std::make_unique<byte_track::BYTETracker>(
    frame_rate_, track_buffer_, track_thresh_, high_thresh_, match_thresh_, use_maj_cls_);

  RCLCPP_INFO(this->get_logger(), "Configuration successful");
  return TrackingNode::CallbackReturn::SUCCESS;
}

TrackingNode::CallbackReturn TrackingNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating tracking node...");
  if (tracked_dets_pub_ && markers_pub_) {
    tracked_dets_pub_->on_activate();
    markers_pub_->on_activate();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Activation failed: nullptr publisher");
    return TrackingNode::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "Activation successful");
  return TrackingNode::CallbackReturn::SUCCESS;
}

TrackingNode::CallbackReturn TrackingNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating tracking node...");
  if (tracked_dets_pub_ && markers_pub_) {
    tracked_dets_pub_->on_deactivate();
    markers_pub_->on_deactivate();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Deactivation failed: nullptr publisher");
    return TrackingNode::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "Deactivation successful");
  return TrackingNode::CallbackReturn::SUCCESS;
}

TrackingNode::CallbackReturn TrackingNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up tracking node...");

  tracked_dets_pub_.reset();
  markers_pub_.reset();
  dets_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  tracker_.reset();

  RCLCPP_INFO(this->get_logger(), "Clean up successful");
  return TrackingNode::CallbackReturn::SUCCESS;
}

TrackingNode::CallbackReturn TrackingNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down tracking node...");

  tracked_dets_pub_.reset();
  markers_pub_.reset();
  dets_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  tracker_.reset();

  RCLCPP_INFO(this->get_logger(), "Shut down successful");
  return TrackingNode::CallbackReturn::SUCCESS;
}

void TrackingNode::initializeParams()
{
  // Declare parameters
  frame_rate_ = this->declare_parameter<int>("frame_rate", 2);
  track_buffer_ = this->declare_parameter<int>("track_buffer", 30);
  track_thresh_ = static_cast<float>(this->declare_parameter<double>("track_thresh", 0.5));
  high_thresh_ = static_cast<float>(this->declare_parameter<double>("high_thresh", 0.6));
  match_thresh_ = static_cast<float>(this->declare_parameter<double>("match_thresh", 1.0));
  use_maj_cls_ = this->declare_parameter<bool>("use_maj_cls", true);
  output_frame_ = this->declare_parameter<std::string>("output_frame", "map");
  publish_visualization_ = this->declare_parameter<bool>("publish_visualization", false);
}

// Get class id from class_map_ using class name
int TrackingNode::classLookup(const std::string & class_name)
{
  auto it = class_map_.find(class_name);
  if (it != class_map_.end())
    return it->second;
  else {  // Class name key not in map
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      static_logger_, steady_clock, 5000, "Class '%s' not found, defaulting to -1 as id", class_name.c_str());
    return -1;
  }
}

// Get class name from reverse_class_map_ using class id
std::string TrackingNode::reverseClassLookup(int class_id)
{
  auto it = reverse_class_map_.find(class_id);
  if (it != reverse_class_map_.end())
    return it->second;
  else {  // Class id key not in reverse map
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      static_logger_, steady_clock, 5000, "Class %d not found, defaulting to '[unknown]' class", class_id);
    return "[unknown]";
  }
}

// Convert from ros msgs to bytetrack's required format
std::vector<byte_track::Object> TrackingNode::detsToObjects(const vision_msgs::msg::Detection3DArray & dets)
{
  std::vector<byte_track::Object> objs;
  objs.reserve(dets.detections.size());

  for (const auto & det : dets.detections) {
    // Convert from Detection3D to byte_track::Object
    float length = det.bbox.size.x;
    float width = det.bbox.size.y;
    float height = det.bbox.size.z;
    float yaw = tf2::getYaw(det.bbox.center.orientation);
    float x = det.bbox.center.position.x;
    float y = det.bbox.center.position.y;
    float z = det.bbox.center.position.z;

    byte_track::Rect<float> rect{x, y, z, yaw, length, width, height};
    int label = 0;
    float prob = 1.0;

    // Get highest scored class hypothesis, skipping attribute hypotheses (prefixed with "state:", "behavior:", etc.)
    decltype(det.results.begin()) best_hyp = det.results.end();
    for (auto it = det.results.begin(); it != det.results.end(); ++it) {
      const auto & cid = it->hypothesis.class_id;
      if (cid.find(':') != std::string::npos) continue;  // Skip attribute hypotheses
      if (best_hyp == det.results.end() || it->hypothesis.score > best_hyp->hypothesis.score) {
        best_hyp = it;
      }
    }

    if (best_hyp == det.results.end()) {
      RCLCPP_WARN(static_logger_, "det.results must be non-empty, falling back to dummy values");
      label = -1;
      prob = 0.0;
    } else {
      label = classLookup(best_hyp->hypothesis.class_id);
      prob = best_hyp->hypothesis.score;
    }

    byte_track::Object obj(rect, label, prob);
    objs.push_back(obj);
  }

  return objs;
}

// Convert from bytetrack format back to ros msgs
vision_msgs::msg::Detection3DArray TrackingNode::STracksToTracks(
  const std::vector<byte_track::BYTETracker::STrackPtr> & strk_ptrs, const std_msgs::msg::Header & header)
{
  // Use same header as detections for same time stamps
  vision_msgs::msg::Detection3DArray trks;
  trks.header.frame_id = header.frame_id;
  trks.header.stamp = header.stamp;

  for (const auto & strk_ptr : strk_ptrs) {
    // Convert STrackPtr to Detection2D
    auto rect = strk_ptr->getRect();
    auto score = strk_ptr->getScore();
    auto trk_id = strk_ptr->getTrackId();
    auto class_id = strk_ptr->getClassId();

    vision_msgs::msg::Detection3D trk;
    trk.header.frame_id = header.frame_id;
    trk.header.stamp = header.stamp;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.score = score;
    hyp.hypothesis.class_id = reverseClassLookup(class_id);
    trk.results.push_back(hyp);

    tf2::Quaternion q;
    q.setRPY(0, 0, rect.yaw());

    trk.bbox.center.position.x = rect.x();
    trk.bbox.center.position.y = rect.y();
    trk.bbox.center.position.z = rect.z();
    trk.bbox.center.orientation = tf2::toMsg(q);
    trk.bbox.size.x = rect.length();
    trk.bbox.size.y = rect.width();
    trk.bbox.size.z = rect.height();

    trk.id = std::to_string(trk_id);

    trks.detections.push_back(trk);
  }

  return trks;
}

void TrackingNode::detectionsCallback(vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  // Activation guard
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  // Get newest frame transform
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(output_frame_, msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform unavailable: %s", ex.what());
    RCLCPP_WARN(this->get_logger(), "Falling back to identity transform");
    // Fall back to identity
    tf_stamped.header.stamp = msg->header.stamp;
    tf_stamped.header.frame_id = output_frame_;
    tf_stamped.child_frame_id = msg->header.frame_id;
    tf_stamped.transform.translation.x = 0.0;
    tf_stamped.transform.translation.y = 0.0;
    tf_stamped.transform.translation.z = 0.0;
    tf_stamped.transform.rotation.x = 0.0;
    tf_stamped.transform.rotation.y = 0.0;
    tf_stamped.transform.rotation.z = 0.0;
    tf_stamped.transform.rotation.w = 1.0;
  }

  vision_msgs::msg::Detection3DArray tf_msg;
  tf_msg.header.frame_id = output_frame_;
  tf_msg.header.stamp = msg->header.stamp;

  // Transform all dets
  for (const auto & det : msg->detections) {
    vision_msgs::msg::Detection3D tf_det = det;
    tf_det.header.frame_id = output_frame_;
    tf2::doTransform(det.bbox.center, tf_det.bbox.center, tf_stamped);
    tf_msg.detections.push_back(tf_det);
  }

  // Run bytetrack on detections
  auto objs = detsToObjects(tf_msg);
  auto stracks = tracker_->update(objs);
  auto tracked_dets = STracksToTracks(stracks, tf_msg.header);

  // Match each tracked detection to the nearest input detection to:
  // 1. Override ByteTrack's filtered yaw (Kalman filter causes unstable spinning)
  // 2. Carry forward attribute hypotheses (state:*, behavior:*) from enriched detections
  for (auto & trk : tracked_dets.detections) {
    double best_dist = std::numeric_limits<double>::max();
    const vision_msgs::msg::Detection3D * best_det = nullptr;
    for (const auto & det : tf_msg.detections) {
      double dx = trk.bbox.center.position.x - det.bbox.center.position.x;
      double dy = trk.bbox.center.position.y - det.bbox.center.position.y;
      double dz = trk.bbox.center.position.z - det.bbox.center.position.z;
      double dist = dx * dx + dy * dy + dz * dz;
      if (dist < best_dist) {
        best_dist = dist;
        best_det = &det;
      }
    }
    if (best_det) {
      trk.bbox.center.orientation = best_det->bbox.center.orientation;

      // Append attribute hypotheses (class_id contains ':') from the matched input detection
      for (const auto & result : best_det->results) {
        if (result.hypothesis.class_id.find(':') != std::string::npos) {
          trk.results.push_back(result);
        }
      }
    }
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "Received %zd dets, transformed to %zd dets, publishing %zd tracks...",
    msg->detections.size(),
    tf_msg.detections.size(),
    tracked_dets.detections.size());
  tracked_dets_pub_->publish(tracked_dets);

  // Publish visualization markers
  if (publish_visualization_) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Delete all previous markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (const auto & det : tracked_dets.detections) {
      visualization_msgs::msg::Marker marker;
      marker.header = tracked_dets.header;
      marker.ns = "tracked_objects";
      marker.id = std::stoi(det.id);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position = det.bbox.center.position;
      marker.pose.orientation = det.bbox.center.orientation;
      marker.scale.x = std::max(0.1, det.bbox.size.x);
      marker.scale.y = std::max(0.1, det.bbox.size.y);
      marker.scale.z = std::max(0.1, det.bbox.size.z);

      // Color from attribute hypotheses, falling back to deterministic track ID color
      int trk_id = std::stoi(det.id);
      marker.color.r = static_cast<float>((trk_id * 67) % 255) / 255.0f;
      marker.color.g = static_cast<float>((trk_id * 123) % 255) / 255.0f;
      marker.color.b = static_cast<float>((trk_id * 200) % 255) / 255.0f;
      marker.color.a = 0.6f;

      // Find best attribute hypothesis to determine marker color (has to be greater than 50%)
      double best_attr_score = 0.5;
      std::string best_attr;
      for (const auto & result : det.results) {
        const auto & cid = result.hypothesis.class_id;
        if (cid.find(':') != std::string::npos && result.hypothesis.score > best_attr_score) {
          best_attr_score = result.hypothesis.score;
          best_attr = cid;
        }
      }

      if (!best_attr.empty()) {
        if (best_attr == "state:red") {
          marker.color.r = 1.0f;
          marker.color.g = 0.0f;
          marker.color.b = 0.0f;
        } else if (best_attr == "state:yellow") {
          marker.color.r = 1.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
        } else if (best_attr == "state:green") {
          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
        } else if (best_attr == "behavior:braking") {
          marker.color.r = 1.0f;
          marker.color.g = 0.0f;
          marker.color.b = 0.0f;
        } else if (best_attr == "behavior:turning_left" || best_attr == "behavior:turning_right") {
          marker.color.r = 1.0f;
          marker.color.g = 0.7f;
          marker.color.b = 0.0f;
        } else if (best_attr == "behavior:hazard_lights") {
          marker.color.r = 1.0f;
          marker.color.g = 0.5f;
          marker.color.b = 0.0f;
        }
        marker.color.a = 0.8f;
      }
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);

      marker_array.markers.push_back(marker);

      // Text marker for track ID and class
      visualization_msgs::msg::Marker text_marker;
      text_marker.header = tracked_dets.header;
      text_marker.ns = "tracked_labels";
      text_marker.id = trk_id;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;

      text_marker.pose.position = det.bbox.center.position;
      text_marker.pose.position.z += det.bbox.size.z * 0.5 + 0.3;
      text_marker.scale.z = 0.5;

      std::string class_name = det.results.empty() ? "?" : det.results[0].hypothesis.class_id;
      std::string label = class_name + " #" + det.id;
      if (!best_attr.empty()) {
        // Show attribute after the colon (e.g. "car #5 [braking]")
        label += " [" + best_attr.substr(best_attr.find(':') + 1) + "]";
      }
      text_marker.text = label;

      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.b = 1.0f;
      text_marker.color.a = 1.0f;
      text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

      marker_array.markers.push_back(text_marker);
    }

    markers_pub_->publish(marker_array);
  }  // publish_visualization_

  RCLCPP_DEBUG(
    this->get_logger(),
    "First track - Class: '%s'; ID: %d",
    tracked_dets.detections[0].results[0].hypothesis.class_id.c_str(),
    std::stoi(tracked_dets.detections[0].id));
}

RCLCPP_COMPONENTS_REGISTER_NODE(TrackingNode)
