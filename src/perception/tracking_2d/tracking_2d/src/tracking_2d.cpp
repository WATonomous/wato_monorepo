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
  dets_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    kDetectionsTopic, 10, std::bind(&tracking_2d::detectionsCallback, this, std::placeholders::_1));

  // Publishers
  tracked_dets_pub_ = this->create_publisher<tracking_2d_msgs::msg::Tracking2DArray>(kTracksTopic, 10);

  // ByteTrack tracker
  tracker_ =
    std::make_unique<byte_track::BYTETracker>(frame_rate_, track_buffer_, track_thresh_, high_thresh_, match_thresh_);
}

void tracking_2d::initializeParams()
{
  // Declare parameters
  frame_rate_ = this->declare_parameter<int>("frame_rate", 30);
  track_buffer_ = this->declare_parameter<int>("track_buffer", 30);
  track_thresh_ = static_cast<float>(this->declare_parameter<double>("track_thresh", 0.5));
  high_thresh_ = static_cast<float>(this->declare_parameter<double>("high_thresh", 0.6));
  match_thresh_ = static_cast<float>(this->declare_parameter<double>("match_thresh", 0.8));

  RCLCPP_INFO(this->get_logger(), "Parameters initialized");
}

// Convert from ros msgs to bytetrack's required format
std::vector<byte_track::Object> tracking_2d::detsToObjects(const vision_msgs::msg::Detection2DArray::SharedPtr dets)
{
  std::vector<byte_track::Object> objs;
  objs.reserve(dets->detections.size());

  int inc_id = 0;  // Manually set incrementing id (approach subject to change)
  for (const auto & det : dets->detections) {
    // Convert from Detection2D to byte_track::Object
    float width = det.bbox.size_x;
    float height = det.bbox.size_y;
    float x = det.bbox.center.position.x - width / 2;
    float y = det.bbox.center.position.y - height / 2;

    byte_track::Rect<float> rect{x, y, width, height};
    int label;
    float prob;

    byte_track::Object obj(rect, label, prob);

    // Get highest scored hypothesis
    auto best_hyp = std::max_element(
      det.results.begin(),
      det.results.end(),
      [](const vision_msgs::msg::ObjectHypothesisWithPose & a, const vision_msgs::msg::ObjectHypothesisWithPose & b) {
        return a.hypothesis.score < b.hypothesis.score;
      });

    if (best_hyp == det.results.end()) {
      throw std::runtime_error("det.results must be non-empty");
    } else {
      label = inc_id;
      prob = best_hyp->hypothesis.score;
    }

    obj.rect = rect;
    obj.label = label;
    obj.prob = prob;
    objs.push_back(obj);

    ++inc_id;
  }

  return objs;
}

// Convert from bytetrack format back to ros msgs
tracking_2d_msgs::msg::Tracking2DArray tracking_2d::STracksToTracks(
  const std::vector<byte_track::BYTETracker::STrackPtr> & strk_ptrs, const std_msgs::msg::Header & header)
{
  // Use same header as detections for same time stamps
  tracking_2d_msgs::msg::Tracking2DArray trks;
  trks.header.frame_id = header.frame_id;
  trks.header.stamp = header.stamp;

  for (const auto & strk_ptr : strk_ptrs) {
    // Convert STrackPtr to Detection2D
    auto rect = strk_ptr->getRect();
    auto score = strk_ptr->getScore();
    auto trk_id = strk_ptr->getTrackId();

    tracking_2d_msgs::msg::Tracking2D trk;
    trk.header.frame_id = header.frame_id;
    trk.header.stamp = header.stamp;

    trk.id = trk_id;

    trk.cx = rect.x() + rect.width() / 2;
    trk.cy = rect.y() + rect.height() / 2;
    trk.width = rect.width();
    trk.height = rect.height();

    trk.score = score;

    trks.tracks.push_back(trk);
  }

  return trks;
}

void tracking_2d::detectionsCallback(vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  // Run bytetrack on detections
  auto objs = detsToObjects(msg);
  auto stracks = tracker_->update(objs);
  auto tracked_dets = STracksToTracks(stracks, msg->header);

  RCLCPP_DEBUG(this->get_logger(), "Publishing tracked detections...");
  tracked_dets_pub_->publish(tracked_dets);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tracking_2d>());
  rclcpp::shutdown();
  return 0;
}
