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
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class tracking_2d : public rclcpp::Node
{
public:
  tracking_2d();

  static constexpr auto kDetectionsTopic = "input_detections";
  static constexpr auto kTracksTopic = "output_tracks";

private:
  /**
   * @brief Initializes tracker parameters.
   *
   * Sets the ByteTrack tracker's framerate, track buffer, track threshold,
   * high threshold, and max threshold.
  */
  void initializeParams();

  // Callback functions
  /**
   * @brief Callback function for incoming Detection2DArray messages.
   *
   * Uses new detections to update the ByteTrack tracker,
   * performing the necessary type conversions.
   *
   * @param msg The received detections.
  */
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  // Helper functions
  /**
   * @brief Looks up the numerical id corresponding to a class.
   *
   * Looks up the name of the class in the unordered_map of classes.
   * If the name exists in the unordered_map, the corresonding
   * numerical id is returned.
   * If the name does not exist, then the default value 0 is returned
   * and a warning is given.
   *
   * @param class_name The name of the required class.
   * @return int The numerical id corresponding to the class.
   */
  int classLookup(const std::string & class_name);

  /**
   * @brief Converts Detection2DArray messages to ByteTrack Object format.
   *
   * Initializes and populates a vector of ByteTrack Objects using the given
   * Detection2DArray message.
   *
   * For each Detection2D in the Detection2DArray, an Object is populated using:
   * (1) the detection's center xy coordinates, width, and height in pixels; and
   * (2) the label and confidence score of the detection.
   *
   * @param dets The detections to be converted into ByteTrack Objects.
   * @return std::vector<byte_track::Object> The converted ByteTrack Objects.
   *
   * @note Done prior to each tracker update.
   */
  std::vector<byte_track::Object> detsToObjects(const vision_msgs::msg::Detection2DArray::SharedPtr dets);

  /**
   * @brief Converts STrackPtr tracks output by the ByteTrack tracker back into Detection2DArray messages.
   *
   * Initializes and populates a Detection2DArray message containing information on
   * the associated tracks using the given STrackPtr tracks.
   *
   * For each STrackPtr, a Detection2D is populated using:
   * (1) the track's top-left xy coordinates, width, and height in pixels; and
   * (2) the label and confidence score of the track.
   *
   * @param strk_ptrs The Strack shared_ptrs to be converted into a DetectionArray2D ROS2 message.
   * @param header The same header of the detections these tracks are being associated with.
   * @return vision_msgs::msg::Detection2DArray The converted ROS2 message.
   *
   * @note Done after each tracker update.
   */
  vision_msgs::msg::Detection2DArray STracksToTracks(
    const std::vector<byte_track::BYTETracker::STrackPtr> & strk_ptrs, const std_msgs::msg::Header & header);

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr dets_sub_;

  // Publishers
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr tracked_dets_pub_;

  // ByteTrack parameters
  int frame_rate_;
  int track_buffer_;
  float track_thresh_;
  float high_thresh_;
  float match_thresh_;

  std::unordered_map<std::string, int> class_map_;

  std::unique_ptr<byte_track::BYTETracker> tracker_;
};

#endif
