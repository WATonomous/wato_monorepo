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

#include "trajectory_planner/trajectory_visualizer.hpp"

#include <algorithm>
#include <string>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace trajectory_planner
{

TrajectoryVisualizer::TrajectoryVisualizer(
  MarkerArrayPub::SharedPtr marker_pub, MarkerArrayPub::SharedPtr speed_label_pub)
: marker_pub_(marker_pub)
, speed_label_pub_(speed_label_pub)
{}

void TrajectoryVisualizer::publish(const wato_trajectory_msgs::msg::Trajectory & traj, double limit_speed)
{
  const bool publish_arrows = marker_pub_->get_subscription_count() > 0;
  const bool publish_labels = speed_label_pub_->get_subscription_count() > 0;

  if (!publish_arrows && !publish_labels) {
    return;
  }

  visualization_msgs::msg::MarkerArray arrow_markers;
  visualization_msgs::msg::MarkerArray label_markers;

  // Delete all previous markers first
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  if (publish_arrows) {
    arrow_markers.markers.push_back(delete_marker);
  }
  if (publish_labels) {
    label_markers.markers.push_back(delete_marker);
  }

  // Create an arrow marker for each point, colored by speed ratio
  int arrow_id = 0;
  int label_id = 0;
  for (const auto & point : traj.points) {
    double speed_ratio = (limit_speed > 0.0) ? std::max(0.0, std::min(1.0, point.max_speed / limit_speed)) : 0.0;

    if (publish_arrows) {
      visualization_msgs::msg::Marker arrow;
      arrow.header = traj.header;
      arrow.ns = "trajectory_arrows";
      arrow.id = arrow_id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose = point.pose;

      // Small arrows (0.2 m shaft) for dense per-point visualization
      arrow.scale.x = 0.2;
      arrow.scale.y = 0.05;
      arrow.scale.z = 0.10;

      // Color: green (fast) → yellow (mid) → red (slow)
      arrow.color.r = static_cast<float>(1.0 - speed_ratio);
      arrow.color.g = static_cast<float>(speed_ratio);
      arrow.color.b = 0.0f;
      arrow.color.a = 0.8f;

      arrow_markers.markers.push_back(arrow);
    }

    // Only add labels for every 4th point to avoid clutter
    if (publish_labels && (label_id % 4 == 0)) {
      visualization_msgs::msg::Marker label;
      label.header = traj.header;
      label.ns = "trajectory_speed_labels";
      label.id = label_id;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose = point.pose;
      label.pose.position.z += 0.3;
      label.scale.z = 0.4;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 1.0f;

      std::string speed_str = std::to_string(point.max_speed);
      label.text = speed_str.substr(0, speed_str.find(".") + 2) + " m/s";
      label_markers.markers.push_back(label);
    }
    ++label_id;
  }

  if (publish_arrows) {
    marker_pub_->publish(arrow_markers);
  }
  if (publish_labels) {
    speed_label_pub_->publish(label_markers);
  }
}

}  // namespace trajectory_planner
