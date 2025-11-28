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

#ifndef TRACK_EVAL_HPP
#define TRACK_EVAL_HPP

#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "track_eval/Accumulator.hpp"

class track_eval : public rclcpp::Node
{
public:
  track_eval();

  static constexpr auto kDetectionsTopic = "input_detections";
  static constexpr auto kTracksTopic = "input_tracks";

private:
  /// @brief Initializes required parameters for wvaluation.
  void initializeParams();

  /**
   * @brief Callback function for incoming ground truths.
   *
   * Saves the latest ground truths for use when tracks are received.
   *
   * @param msg The received gts.
   */
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  /**
   * @brief Callback function for incoming tracks.
   *
   * Saves the latest tracks and attempts to evaluate
   * using the latest tracks and detections.
   *
   * @param msg The received tracks.
   */
  void tracksCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  /**
   * @brief Attempts to evaluate on newest ground truths and tracks.
   *
   * If valid non-empty detections and tracks have been received,
   * then evaluation proceeds.
   * Otherwise, returns immediately and a warning is given.
   */
  void tryMetrics();

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr dets_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr trks_sub_;

  // Parameters
  int precision_;

  // Other values
  int timestep_;

  vision_msgs::msg::Detection2DArray::SharedPtr latest_dets_;
  vision_msgs::msg::Detection2DArray::SharedPtr latest_trks_;

  Accumulator acc_;

  std::vector<int> row_matches_, col_matches_;
};

#endif
