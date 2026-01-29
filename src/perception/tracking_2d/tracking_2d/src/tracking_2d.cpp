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

#include "tracking_2d/tracking_2d.hpp"
#include <memory>
#include <string>
#include <vector>

tracking_2d::tracking_2d()
: Node("tracking_2d")
{
  initializeParams();

  // Subscribers
  dets_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    kDetectionsTopic, 10, std::bind(&tracking_2d::detectionsCallback, this, std::placeholders::_1));

  // Publishers
  tracked_dets_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(kTracksTopic, 10);

  // ByteTrack tracker
  tracker_ =
    std::make_unique<byte_track::BYTETracker>(frame_rate_, track_buffer_, track_thresh_, high_thresh_, match_thresh_);
}

void tracking_2d::initializeParams()
{
  // Declare parameters
  frame_rate_ = this->declare_parameter<int>("frame_rate", 2);
  track_buffer_ = this->declare_parameter<int>("track_buffer", 30);
  track_thresh_ = static_cast<float>(this->declare_parameter<double>("track_thresh", 0.5));
  high_thresh_ = static_cast<float>(this->declare_parameter<double>("high_thresh", 0.6));
  match_thresh_ = static_cast<float>(this->declare_parameter<double>("match_thresh", 1.0));

  class_map_ = {
    {"car", 0}, {"truck", 1}, {"bicycle", 2}, {"pedestrian", 3}, {"bus", 4},
    // etc...
  };

  for (const auto & [key, val] : class_map_) reverse_class_map_[val] = key;

  RCLCPP_INFO(this->get_logger(), "Parameters initialized");
}

// Get class id from class_map_ using class name
int tracking_2d::classLookup(const std::string & class_name)
{
  auto it = class_map_.find(class_name);
  if (it != class_map_.end())
    return it->second;
  else {  // Class name key not in map
    RCLCPP_WARN(this->get_logger(), "Class '%s' not found, defaulting to -1 as id", class_name.c_str());
    return -1;
  }
}

// Get class name from reverse_class_map_ using class id
std::string tracking_2d::reverseClassLookup(int class_id)
{
  auto it = reverse_class_map_.find(class_id);
  if (it != reverse_class_map_.end())
    return it->second;
  else {  // Class id key not in reverse map
    RCLCPP_WARN(this->get_logger(), "Class %d not found, defaulting to '[unknown]' class", class_id);
    return "[unknown]";
  }
}

// Convert from ros msgs to bytetrack's required format
std::vector<byte_track::Object> tracking_2d::detsToObjects(const vision_msgs::msg::Detection3DArray::SharedPtr dets)
{
  std::vector<byte_track::Object> objs;
  objs.reserve(dets->detections.size());

  for (const auto & det : dets->detections) {
    // Convert from Detection2D to byte_track::Object
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

    // Get highest scored hypothesis
    // auto best_hyp = std::max_element(
    //   det.results.begin(),
    //   det.results.end(),
    //   [](const vision_msgs::msg::ObjectHypothesisWithPose & a, const vision_msgs::msg::ObjectHypothesisWithPose & b) {
    //     return a.hypothesis.score < b.hypothesis.score;
    //   });

    // if (best_hyp == det.results.end()) {
    //   RCLCPP_WARN(this->get_logger(), "det.results must be non-empty, falling back to dummy values");
    //   label = -1;
    //   prob = 0.0;
    // } else {
    //   label = classLookup(best_hyp->hypothesis.class_id);
    //   prob = best_hyp->hypothesis.score;
    // }

    byte_track::Object obj(rect, label, prob);
    objs.push_back(obj);
  }

  return objs;
}

// Convert from bytetrack format back to ros msgs
vision_msgs::msg::Detection3DArray tracking_2d::STracksToTracks(
  const std::vector<byte_track::BYTETracker::STrackPtr> & strk_ptrs, const std_msgs::msg::Header & header)
{
  // Use same header as detections for same time stamps
  vision_msgs::msg::Detection3DArray trks;
  trks.header.frame_id = header.frame_id;
  trks.header.stamp = header.stamp;
  // std_msgs::msg::ColorRGBA col;
  // col.r = 0.0;
  // col.b = 0.0;
  // col.g = 1.0;
  // col.a = 1.0;
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

    trk.bbox.center.position.x = rect.x();
    trk.bbox.center.position.y = rect.y();
    trk.bbox.center.position.z = rect.z();
    trk.bbox.center.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(rect.yaw()/2), std::cos(rect.yaw()/2)));
    trk.bbox.size.x = rect.length();
    trk.bbox.size.y = rect.width();
    trk.bbox.size.z = rect.height();

    trk.id = trk_id;
    // trk.type = trk.CUBE;
    // trk.color = col;
    // trk.action = trk.ADD;

    trks.detections.push_back(trk);
  }

  return trks;
}

void tracking_2d::detectionsCallback(vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  // Run bytetrack on detections
  auto objs = detsToObjects(msg);
  auto stracks = tracker_->update(objs);
  auto tracked_dets = STracksToTracks(stracks, msg->detections[0].header);

  RCLCPP_INFO(this->get_logger(), "Received %zd dets, publishing %zd tracks...", msg->detections.size(), tracked_dets.detections.size());
  tracked_dets_pub_->publish(tracked_dets);
  // RCLCPP_DEBUG(
  //   this->get_logger(),
  //   "First track - Class: '%s'; ID: %d",
  //   tracked_dets.detections[0].results[0].hypothesis.class_id.c_str(),
  //   std::stoi(tracked_dets.detections[0].id));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tracking_2d>());
  rclcpp::shutdown();
  return 0;
}
