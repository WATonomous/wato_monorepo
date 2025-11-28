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

#include "track_eval/track_eval.hpp"

#include <string>
#include <vector>

#include "track_eval/Accumulator.hpp"
#include "track_eval/cost.hpp"
#include "track_eval/lap.hpp"
#include "track_eval/Matches.hpp"

track_eval::track_eval()
: Node("track_eval")
{
  initializeParams();

  // Subscribers
  dets_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    kGTsTopic, 10, std::bind(&track_eval::gtsCallback, this, std::placeholders::_1));
  trks_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    kTracksTopic, 10, std::bind(&track_eval::tracksCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Evaluator initialized");
}

void track_eval::initializeParams()
{
  // Declare parameters
  precision_ = this->declare_parameter<int>("precision", 4);

  timestep_ = 0;

  latest_dets_ = nullptr;
  latest_trks_ = nullptr;
}

// Attempt box drawing
void track_eval::tryMetrics()
{
  // Check if ground truths or tracks are missing for whatever reason
  if (!latest_trks_ || !latest_dets_ || latest_trks_->detections.empty() || latest_dets_->detections.empty()) {
    RCLCPP_WARN(this->get_logger(), "Missing ground truths or tracks");
    return;
  }
  std::vector<vision_msgs::msg::Detection2D> dets = latest_dets_->detections;
  std::vector<vision_msgs::msg::Detection2D> trks = latest_trks_->detections;

  auto cost = costMtx(dets, trks);
  double opt = lapjv(cost, row_matches_, col_matches_, 0.8);

  acc_.update(Matches(opt, row_matches_, col_matches_, dets, trks));

  RCLCPP_INFO(this->get_logger(), "---------------------");
  RCLCPP_INFO(this->get_logger(), "Timestep: %d", timestep_);
  RCLCPP_INFO(this->get_logger(), "---------------------");
  RCLCPP_INFO(this->get_logger(), "MOTP: %.*f", precision_, acc_.MOTP());
  RCLCPP_INFO(this->get_logger(), "MOTA: %.*f", precision_, acc_.MOTA());
  RCLCPP_INFO(this->get_logger(), "IDF1: %.*f", precision_, acc_.IDF1());
  RCLCPP_INFO(this->get_logger(), "IDP: %.*f", precision_, acc_.IDP());
  RCLCPP_INFO(this->get_logger(), "IDR: %.*f", precision_, acc_.IDR());
  RCLCPP_INFO(this->get_logger(), "FP: %d", acc_.getTotFP());
  RCLCPP_INFO(this->get_logger(), "FN: %d", acc_.getTotFN());
  RCLCPP_INFO(this->get_logger(), "IDSW: %d", acc_.getTotIDSW());
  RCLCPP_INFO(this->get_logger(), "# gts: %d", acc_.getTotGts());
  RCLCPP_INFO(this->get_logger(), "# tracks: %d", acc_.getTotTrks());
  ++timestep_;
}

// Save most recent ground truths
void track_eval::gtsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  latest_dets_ = msg;
}

// Image annotation triggered upon receiving tracks
void track_eval::tracksCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  latest_trks_ = msg;
  tryMetrics();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<track_eval>());
  rclcpp::shutdown();
  return 0;
}
