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

#ifndef TRACKING_2D_HPP
#define TRACKING_2D_HPP

#include <ByteTrack/BYTETracker.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tracking_2d_msgs/msg/tracking2_d_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class tracking_2d : public rclcpp::Node
{
public:
  tracking_2d();

  static constexpr auto kDetectionsTopic = "input_detections";
  static constexpr auto kTracksTopic = "output_tracks";

private:
  void initializeParams();
  /*
   * Initialize tracker parameters.
  */

  // Callback functions
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  /*
   * Callback function for incoming Detection2DArray messages.
   * Uses new detections to update the ByteTrack tracker,
   * performing the necessary type conversions.
  */

  // Helper functions
  std::vector<byte_track::Object> detsToObjects(const vision_msgs::msg::Detection2DArray::SharedPtr dets);
  /*
   * Converts Detection2DArray messages to ByteTrack Object format.
   * Done prior to each tracker update.
  */
  tracking_2d_msgs::msg::Tracking2DArray STracksToTracks(
    const std::vector<byte_track::BYTETracker::STrackPtr> & strk_ptrs, const std_msgs::msg::Header & header);
  /*
   * Converts STrackPtr tracks that are output by byteTrack into Track2DArray messages.
   * Done after each tracker update.
  */

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr dets_sub_;

  // Publishers
  rclcpp::Publisher<tracking_2d_msgs::msg::Tracking2DArray>::SharedPtr tracked_dets_pub_;

  // ByteTrack parameters
  int frame_rate_;
  int track_buffer_;
  float track_thresh_;
  float high_thresh_;
  float match_thresh_;

  std::unique_ptr<byte_track::BYTETracker> tracker_;
};

#endif
