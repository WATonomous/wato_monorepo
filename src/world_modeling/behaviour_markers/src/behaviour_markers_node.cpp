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

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "lanelet_markers/marker_utils.hpp"

namespace behaviour_markers
{

  BehaviourMarkersNode::BehaviourMarkersNode()
      : Node("behaviour_markers_node")
  {
    // Declare and get visual parameters
    this->declare_parameter("frame_id", "map");
    this->declare_parameter("preferred_centerline_line_width", 0.45);
    this->declare_parameter("preferred_boundary_line_width", 0.18);
    this->declare_parameter("current_lane_line_width", 0.2);
    this->declare_parameter("text_height", 0.8);
    this->declare_parameter("show_boundaries", true);
    this->declare_parameter("show_lanelet_ids", false);
    this->declare_parameter("show_current_lane", true);
    this->declare_parameter("show_info", true);

    frame_id_ = this->get_parameter("frame_id").as_string();
    preferred_centerline_line_width_ = this->get_parameter("preferred_centerline_line_width").as_double();
    preferred_boundary_line_width_ = this->get_parameter("preferred_boundary_line_width").as_double();
    current_lane_line_width_ = this->get_parameter("current_lane_line_width").as_double();
    text_height_ = this->get_parameter("text_height").as_double();
    show_boundaries_ = this->get_parameter("show_boundaries").as_bool();
    show_lanelet_ids_ = this->get_parameter("show_lanelet_ids").as_bool();
    show_current_lane_ = this->get_parameter("show_current_lane").as_bool();
    show_info_ = this->get_parameter("show_info").as_bool();

    // Publishers and subscribers use relative topic names for remapping
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

    execute_behaviour_sub_ = this->create_subscription<behaviour_msgs::msg::ExecuteBehaviour>(
        "execute_behaviour",
        10,
        std::bind(&BehaviourMarkersNode::executeBehaviourCallback, this, std::placeholders::_1));

    route_ahead_sub_ = this->create_subscription<lanelet_msgs::msg::RouteAhead>(
        "route_ahead", 10, std::bind(&BehaviourMarkersNode::routeAheadCallback, this, std::placeholders::_1));

    lanelet_ahead_sub_ = this->create_subscription<lanelet_msgs::msg::LaneletAhead>(
        "lanelet_ahead", 10, std::bind(&BehaviourMarkersNode::laneletAheadCallback, this, std::placeholders::_1));

    lane_context_sub_ = this->create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
        "lane_context", 10, std::bind(&BehaviourMarkersNode::laneContextCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "BehaviourMarkersNode initialized");
  }

  void BehaviourMarkersNode::executeBehaviourCallback(const behaviour_msgs::msg::ExecuteBehaviour::SharedPtr msg)
  {
    latest_execute_behaviour_ = *msg;
    has_latest_execute_behaviour_ = true;
    publishMarkers();
  }

  void BehaviourMarkersNode::routeAheadCallback(const lanelet_msgs::msg::RouteAhead::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty())
    {
      frame_hint_ = msg->header.frame_id;
    }
    cacheLanelets(msg->lanelets);
    if (has_latest_execute_behaviour_)
    {
      publishMarkers();
    }
  }

  void BehaviourMarkersNode::laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty())
    {
      frame_hint_ = msg->header.frame_id;
    }
    cacheLanelets(msg->lanelets);
    if (has_latest_execute_behaviour_)
    {
      publishMarkers();
    }
  }

  void BehaviourMarkersNode::laneContextCallback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty())
    {
      frame_hint_ = msg->header.frame_id;
    }
    current_lanelet_ = msg->current_lanelet;
    has_current_lane_ = true;
    cacheLanelets({msg->current_lanelet});
    if (has_latest_execute_behaviour_)
    {
      publishMarkers();
    }
  }

  void BehaviourMarkersNode::cacheLanelets(const std::vector<lanelet_msgs::msg::Lanelet> &lanelets)
  {
    for (const auto &lanelet : lanelets)
    {
      if (lanelet.id > 0)
      {
        lanelet_cache_[lanelet.id] = lanelet;
      }
    }
  }

  std::string BehaviourMarkersNode::toLower(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c)
                   { return static_cast<char>(std::tolower(c)); });
    return text;
  }

  std_msgs::msg::ColorRGBA BehaviourMarkersNode::colorForBehaviour(const std::string &behaviour) const
  {
    const std::string normalized = toLower(behaviour);
    if (normalized.find("lane change left") != std::string::npos)
    {
      return lanelet_markers::makeColor(1.0f, 0.55f, 0.1f, 0.95f);
    }
    if (normalized.find("lane change right") != std::string::npos)
    {
      return lanelet_markers::makeColor(1.0f, 0.1f, 0.9f, 0.95f);
    }
    if (normalized.find("follow route") != std::string::npos)
    {
      return lanelet_markers::makeColor(0.0f, 0.8f, 1.0f, 0.95f);
    }
    if (normalized.find("follow lane") != std::string::npos)
    {
      return lanelet_markers::makeColor(0.15f, 0.95f, 0.35f, 0.95f);
    }
    if (normalized.find("standby") != std::string::npos)
    {
      return lanelet_markers::makeColor(0.8f, 0.8f, 0.8f, 0.75f);
    }
    return lanelet_markers::makeColor(1.0f, 1.0f, 1.0f, 0.9f);
  }

  void BehaviourMarkersNode::publishMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int32_t marker_id = 0;
    const auto stamp = this->get_clock()->now();

    const std::string frame = frame_id_.empty() ? (frame_hint_.empty() ? "map" : frame_hint_) : frame_id_;

    const std::vector<std::string> namespaces = {
        "bt_pref_centerlines",
        "bt_pref_boundaries",
        "bt_pref_ids",
        "bt_current_lane",
        "bt_behaviour_text",
        "bt_unresolved_ids"};

    for (const auto &ns : namespaces)
    {
      auto delete_marker = lanelet_markers::createDeleteAllMarker(ns, frame);
      delete_marker.header.stamp = stamp;
      marker_array.markers.push_back(delete_marker);
    }

    if (!has_latest_execute_behaviour_)
    {
      markers_pub_->publish(marker_array);
      return;
    }

    const auto behaviour_color = colorForBehaviour(latest_execute_behaviour_.behaviour);

    int resolved_count = 0;
    std::vector<int64_t> unresolved_ids;
    geometry_msgs::msg::Point anchor_point;
    bool has_anchor_point = false;

    for (const auto lanelet_id : latest_execute_behaviour_.preferred_lanelet_ids)
    {
      const auto it = lanelet_cache_.find(lanelet_id);
      if (it == lanelet_cache_.end())
      {
        unresolved_ids.push_back(lanelet_id);
        continue;
      }

      const auto &lanelet = it->second;
      ++resolved_count;

      if (!has_anchor_point && !lanelet.centerline.empty())
      {
        anchor_point = lanelet.centerline.front();
        has_anchor_point = true;
      }

      if (lanelet.centerline.size() >= 2)
      {
        auto centerline_marker = lanelet_markers::createLineStripMarker(
            "bt_pref_centerlines",
            marker_id++,
            frame,
            lanelet.centerline,
            behaviour_color,
            preferred_centerline_line_width_);
        centerline_marker.header.stamp = stamp;
        marker_array.markers.push_back(centerline_marker);

        if (show_lanelet_ids_)
        {
          const size_t mid = lanelet.centerline.size() / 2;
          geometry_msgs::msg::Point text_pos = lanelet.centerline[mid];
          text_pos.z += 1.0;

          auto id_marker = lanelet_markers::createTextMarker(
              "bt_pref_ids",
              marker_id++,
              frame,
              text_pos,
              std::to_string(lanelet.id),
              lanelet_markers::makeColor(1.0f, 1.0f, 1.0f, 0.95f),
              0.5);
          id_marker.header.stamp = stamp;
          marker_array.markers.push_back(id_marker);
        }
      }

      if (show_boundaries_)
      {
        auto boundary_color = behaviour_color;
        boundary_color.a = std::max(0.45f, behaviour_color.a * 0.7f);

        if (lanelet.left_boundary.size() >= 2)
        {
          auto left_marker = lanelet_markers::createDottedLineMarker(
              "bt_pref_boundaries",
              marker_id++,
              frame,
              lanelet.left_boundary,
              boundary_color,
              preferred_boundary_line_width_,
              1.5);
          left_marker.header.stamp = stamp;
          marker_array.markers.push_back(left_marker);
        }
        if (lanelet.right_boundary.size() >= 2)
        {
          auto right_marker = lanelet_markers::createDottedLineMarker(
              "bt_pref_boundaries",
              marker_id++,
              frame,
              lanelet.right_boundary,
              boundary_color,
              preferred_boundary_line_width_,
              1.5);
          right_marker.header.stamp = stamp;
          marker_array.markers.push_back(right_marker);
        }
      }
    }

    if (show_current_lane_ && has_current_lane_ && current_lanelet_.centerline.size() >= 2)
    {
      auto current_marker = lanelet_markers::createLineStripMarker(
          "bt_current_lane",
          marker_id++,
          frame,
          current_lanelet_.centerline,
          lanelet_markers::makeColor(0.25f, 0.95f, 0.25f, 0.75f),
          current_lane_line_width_);
      current_marker.header.stamp = stamp;
      marker_array.markers.push_back(current_marker);

      if (!has_anchor_point && !current_lanelet_.centerline.empty())
      {
        anchor_point = current_lanelet_.centerline.front();
        has_anchor_point = true;
      }
    }

    if (show_info_)
    {
      if (!has_anchor_point)
      {
        anchor_point.x = 0.0;
        anchor_point.y = 0.0;
        anchor_point.z = 0.0;
      }

      geometry_msgs::msg::Point info_pos = anchor_point;
      info_pos.z += 2.5;

      std::ostringstream info;
      info << "BT: " << latest_execute_behaviour_.behaviour;
      info << " | preferred: " << latest_execute_behaviour_.preferred_lanelet_ids.size();
      info << " | resolved: " << resolved_count;

      auto info_marker = lanelet_markers::createTextMarker(
          "bt_behaviour_text",
          marker_id++,
          frame,
          info_pos,
          info.str(),
          lanelet_markers::makeColor(1.0f, 1.0f, 1.0f, 0.95f),
          text_height_);
      info_marker.header.stamp = stamp;
      marker_array.markers.push_back(info_marker);

      if (!unresolved_ids.empty())
      {
        geometry_msgs::msg::Point missing_pos = info_pos;
        missing_pos.z += 0.8;

        std::ostringstream missing;
        missing << "Missing lanelets: ";
        for (size_t i = 0; i < unresolved_ids.size(); ++i)
        {
          if (i > 0)
          {
            missing << ", ";
          }
          missing << unresolved_ids[i];
          if (i >= 4 && unresolved_ids.size() > 5)
          {
            missing << ", ...";
            break;
          }
        }

        auto missing_marker = lanelet_markers::createTextMarker(
            "bt_unresolved_ids",
            marker_id++,
            frame,
            missing_pos,
            missing.str(),
            lanelet_markers::makeColor(1.0f, 0.2f, 0.2f, 0.95f),
            text_height_ * 0.8);
        missing_marker.header.stamp = stamp;
        marker_array.markers.push_back(missing_marker);

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Missing lanelet geometry for %zu preferred lanelet IDs",
            unresolved_ids.size());
      }
    }

    markers_pub_->publish(marker_array);
  }

} // namespace behaviour_markers

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<behaviour_markers::BehaviourMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
