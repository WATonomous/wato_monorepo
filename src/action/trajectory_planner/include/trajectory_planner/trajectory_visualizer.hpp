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

#pragma once

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace trajectory_planner
{

class TrajectoryVisualizer
{
public:
  using MarkerArrayPub = rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>;

  explicit TrajectoryVisualizer(MarkerArrayPub::SharedPtr marker_pub, MarkerArrayPub::SharedPtr speed_label_pub);

  // Builds and publishes MarkerArray for the given trajectory.
  // limit_speed is used to normalise arrow colors (green = fast, red = slow).
  // Arrows go to marker_pub_; speed labels go to speed_label_pub_ (separate toggle in RViz).
  void publish(const wato_trajectory_msgs::msg::Trajectory & traj, double limit_speed);

private:
  MarkerArrayPub::SharedPtr marker_pub_;
  MarkerArrayPub::SharedPtr speed_label_pub_;
};

}  // namespace trajectory_planner
