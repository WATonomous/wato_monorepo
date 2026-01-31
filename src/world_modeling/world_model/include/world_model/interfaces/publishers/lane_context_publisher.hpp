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

#ifndef WORLD_MODEL__INTERFACES__PUBLISHERS__LANE_CONTEXT_PUBLISHER_HPP_
#define WORLD_MODEL__INTERFACES__PUBLISHERS__LANE_CONTEXT_PUBLISHER_HPP_

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Publishes current lane context at a fixed rate.
 *
 * Queries TF for ego pose and LaneletHandler for lane context
 * (current lanelet, distances to events, etc.) and publishes periodically.
 */
class LaneContextPublisher : public InterfaceBase
{
public:
  LaneContextPublisher(
    rclcpp_lifecycle::LifecycleNode * node, const LaneletHandler * lanelet_handler, tf2_ros::Buffer * tf_buffer)
  : node_(node)
  , lanelet_(lanelet_handler)
  , ego_pose_(tf_buffer, node->get_parameter("map_frame").as_string(), node->get_parameter("base_frame").as_string())
  {
    rate_hz_ = node_->declare_parameter<double>("lane_context_publish_rate_hz", 10.0);
    route_priority_threshold_m_ = node_->declare_parameter<double>("lane_context_route_priority_threshold_m", 10.0);
    heading_search_radius_m_ = node_->declare_parameter<double>("lane_context_heading_search_radius_m", 15.0);

    pub_ = node_->create_publisher<lanelet_msgs::msg::CurrentLaneContext>("lane_context", 10);
  }

  void activate() override
  {
    pub_->on_activate();

    if (rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / rate_hz_);
      timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&LaneContextPublisher::publish, this));
    }
  }

  void deactivate() override
  {
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
    pub_->on_deactivate();
  }

private:
  /**
   * @brief Extract yaw (heading) from quaternion orientation.
   */
  static double extractYaw(const geometry_msgs::msg::Quaternion & q)
  {
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  /**
   * @brief Timer callback that publishes current lane context.
   *
   * Looks up ego pose via TF, finds the current lanelet using heading-aligned
   * search with a BFS hint from the previous tick, and publishes lane context
   * including distances to upcoming events. Caches the lanelet and rebuilds
   * the static context only when the current lanelet changes.
   */
  void publish()
  {
    if (!lanelet_->isMapLoaded()) {
      return;
    }

    auto ego = ego_pose_.getEgoPose();
    if (!ego.has_value()) {
      return;
    }

    auto ego_point = ego_pose_.getEgoPoint();

    // Extract yaw from quaternion for heading-aligned lanelet finding
    double yaw = extractYaw(ego->pose.orientation);

    // Use route-aware + heading-aligned lanelet finding (pass cached lanelet as BFS hint)
    auto current_id = lanelet_->findCurrentLaneletId(
      *ego_point, yaw, route_priority_threshold_m_, heading_search_radius_m_, cached_lanelet_id_);
    if (!current_id.has_value()) {
      return;
    }

    // Check if lanelet changed - rebuild if so
    if (!cached_lanelet_id_.has_value() || *cached_lanelet_id_ != *current_id) {
      cached_lanelet_id_ = *current_id;
      auto current_ll = lanelet_->getLaneletById(*current_id);
      if (current_ll.has_value()) {
        rebuildContext(*current_ll);
      }
    }

    // Update dynamic values
    updateDynamicContext(*ego);

    pub_->publish(cached_context_);
  }

  /**
   * @brief Rebuilds the cached lane context for a new lanelet.
   *
   * Converts the lanelet to a message and initializes event distances
   * (traffic light, stop line, intersection, yield) from its regulatory elements.
   *
   * @param lanelet The new current lanelet to build context from.
   */
  void rebuildContext(const lanelet::ConstLanelet & lanelet)
  {
    cached_context_ = lanelet_msgs::msg::CurrentLaneContext();
    cached_context_.current_lanelet = lanelet_->toLaneletMsg(lanelet);

    // Set distances to events based on regulatory elements
    cached_context_.distance_to_intersection_m = -1.0;
    cached_context_.distance_to_traffic_light_m = -1.0;
    cached_context_.distance_to_stop_line_m = -1.0;
    cached_context_.distance_to_yield_m = -1.0;

    for (const auto & reg_elem : cached_context_.current_lanelet.regulatory_elements) {
      if (reg_elem.subtype == "traffic_light") {
        cached_context_.distance_to_traffic_light_m = 0.0;
      }
      if (!reg_elem.ref_lines.empty()) {
        cached_context_.distance_to_stop_line_m = 0.0;
      }
    }
    if (cached_context_.current_lanelet.is_intersection) {
      cached_context_.distance_to_intersection_m = 0.0;
    }
  }

  /**
   * @brief Updates frame-by-frame dynamic fields in the cached context.
   *
   * Sets the message timestamp and computes the distance from ego to the
   * end of the current lanelet by measuring arc length along the centerline.
   *
   * @param ego Current ego pose in the map frame.
   */
  void updateDynamicContext(const geometry_msgs::msg::PoseStamped & ego)
  {
    cached_context_.header.stamp = node_->get_clock()->now();
    cached_context_.header.frame_id = ego_pose_.mapFrame();

    // TODO(WATonomous): Calculate arc_length, lateral_offset, heading_error
    cached_context_.arc_length = 0.0;
    cached_context_.lateral_offset = 0.0;
    cached_context_.heading_error = 0.0;

    // Calculate distance to lanelet end using arc length along centerline
    const auto & cl = cached_context_.current_lanelet.centerline;
    if (cl.size() >= 2) {
      double min_dist_sq = std::numeric_limits<double>::max();
      size_t closest_idx = 0;
      for (size_t i = 0; i < cl.size(); ++i) {
        double dx = cl[i].x - ego.pose.position.x;
        double dy = cl[i].y - ego.pose.position.y;
        double d = dx * dx + dy * dy;
        if (d < min_dist_sq) {
          min_dist_sq = d;
          closest_idx = i;
        }
      }
      // Sum arc length from closest point to end
      double arc_len = 0.0;
      for (size_t i = closest_idx; i + 1 < cl.size(); ++i) {
        double dx = cl[i + 1].x - cl[i].x;
        double dy = cl[i + 1].y - cl[i].y;
        arc_len += std::sqrt(dx * dx + dy * dy);
      }
      cached_context_.distance_to_lanelet_end_m = arc_len;
    }
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  const LaneletHandler * lanelet_;
  EgoPoseHelper ego_pose_;
  double rate_hz_;
  double route_priority_threshold_m_;
  double heading_search_radius_m_;

  rclcpp_lifecycle::LifecyclePublisher<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<int64_t> cached_lanelet_id_;
  lanelet_msgs::msg::CurrentLaneContext cached_context_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__PUBLISHERS__LANE_CONTEXT_PUBLISHER_HPP_
