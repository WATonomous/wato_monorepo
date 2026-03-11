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

#include "behaviour_markers/behaviour_markers_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lanelet_markers/marker_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace behaviour_markers
{

BehaviourMarkersNode::BehaviourMarkersNode()
: Node("behaviour_markers_node")
{
  // Declare parameters
  declare_parameter("map_frame", "map");
  declare_parameter("base_frame", "base_link");
  declare_parameter("centerline_width", 0.4);
  declare_parameter("text_height", 1.0);
  declare_parameter("text_offset_z", 3.0);

  map_frame_ = get_parameter("map_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  centerline_width_ = get_parameter("centerline_width").as_double();
  text_height_ = get_parameter("text_height").as_double();
  text_offset_z_ = get_parameter("text_offset_z").as_double();

  // TF for ego position
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publisher
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

  // Subscribers
  execute_behaviour_sub_ = create_subscription<behaviour_msgs::msg::ExecuteBehaviour>(
    "execute_behaviour", 10, std::bind(&BehaviourMarkersNode::executeBehaviourCallback, this, std::placeholders::_1));

  lanelet_ahead_sub_ = create_subscription<lanelet_msgs::msg::LaneletAhead>(
    "lanelet_ahead", 10, std::bind(&BehaviourMarkersNode::laneletAheadCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "BehaviourMarkersNode initialized");
}

void BehaviourMarkersNode::executeBehaviourCallback(const behaviour_msgs::msg::ExecuteBehaviour::SharedPtr msg)
{
  latest_behaviour_ = *msg;
  has_behaviour_ = true;
  publishMarkers();
}

void BehaviourMarkersNode::laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg)
{
  // Cache lanelet geometry from lanelet_ahead
  ahead_lanelets_.clear();
  for (const auto & lanelet : msg->lanelets) {
    if (lanelet.id > 0) {
      ahead_lanelets_[lanelet.id] = lanelet;
    }
  }

  if (has_behaviour_) {
    publishMarkers();
  }
}

void BehaviourMarkersNode::publishMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  const auto stamp = get_clock()->now();
  int32_t marker_id = 0;

  // Delete all previous markers
  const std::vector<std::string> namespaces = {"bt_behaviour", "bt_preferred"};
  for (const auto & ns : namespaces) {
    auto delete_marker = lanelet_markers::createDeleteAllMarker(ns, map_frame_);
    delete_marker.header.stamp = stamp;
    marker_array.markers.push_back(delete_marker);
  }

  if (!has_behaviour_) {
    markers_pub_->publish(marker_array);
    return;
  }

  // Get ego position from TF
  geometry_msgs::msg::Point ego_position;
  bool has_ego_position = false;
  try {
    auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    ego_position.x = transform.transform.translation.x;
    ego_position.y = transform.transform.translation.y;
    ego_position.z = transform.transform.translation.z;
    has_ego_position = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
  }

  // Create text marker above ego (white text)
  if (has_ego_position) {
    geometry_msgs::msg::Point text_pos = ego_position;
    text_pos.z += text_offset_z_;

    const auto text_color = lanelet_markers::makeColor(1.0f, 1.0f, 1.0f, 0.95f);
    auto text_marker = lanelet_markers::createTextMarker(
      "bt_behaviour", marker_id++, map_frame_, text_pos, latest_behaviour_.behaviour, text_color, text_height_);
    text_marker.header.stamp = stamp;
    marker_array.markers.push_back(text_marker);
  }

  // Only show preferred lanelets if there are any
  if (!latest_behaviour_.preferred_lanelet_ids.empty()) {
    const auto preferred_color = lanelet_markers::makeColor(1.0f, 0.2f, 0.2f, 0.9f);  // Red for preferred

    // Draw only preferred lanelets
    for (const auto & lanelet_id : latest_behaviour_.preferred_lanelet_ids) {
      auto it = ahead_lanelets_.find(lanelet_id);
      if (it == ahead_lanelets_.end() || it->second.centerline.size() < 2) {
        continue;
      }

      auto centerline_marker = lanelet_markers::createLineStripMarker(
        "bt_preferred", marker_id++, map_frame_, it->second.centerline, preferred_color, centerline_width_);
      centerline_marker.header.stamp = stamp;
      marker_array.markers.push_back(centerline_marker);
    }
  }

  markers_pub_->publish(marker_array);
}

}  // namespace behaviour_markers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<behaviour_markers::BehaviourMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
