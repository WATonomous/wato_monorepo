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
    route_lookahead_m_ = node_->declare_parameter<double>("lane_context_route_lookahead_m", 100.0);
    route_priority_threshold_m_ = node_->declare_parameter<double>("lane_context_route_priority_threshold_m", 1);
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

    // Calculate distance to lanelet end using the shared lanelet-handler utility.
    auto current_lanelet = lanelet_->getLaneletById(cached_context_.current_lanelet.id);
    if (current_lanelet.has_value()) {
      cached_context_.distance_to_lanelet_end_m =
        lanelet_->getRemainingDistanceToLaneletEnd(*current_lanelet, ego.pose.position);
    } else {
      cached_context_.distance_to_lanelet_end_m = 0.0;
    }

    cached_context_.distance_to_intersection_m =
      cached_context_.current_lanelet.is_intersection ? 0.0 : -1.0;

    // Populate upcoming lanelets and distances using a fixed ego-relative route lookahead.
    auto route_ahead = lanelet_->getRouteAhead(ego.pose.position, route_lookahead_m_);
    cached_context_.upcoming_lanelet_ids = route_ahead.ids;
    cached_context_.upcoming_lanelet_distances_m.clear();

    // Find current lanelet index in the route to calculate relative distances
    int current_idx = -1;
    for (size_t i = 0; i < route_ahead.ids.size(); ++i) {
      if (route_ahead.ids[i] == cached_context_.current_lanelet.id) {
        current_idx = static_cast<int>(i);
        break;
      }
    }

    if (current_idx >= 0) {
      // Calculate distances relative to ego
      // Start with distance to end of current lanelet (which is start of next)
      double dist_accumulator = cached_context_.distance_to_lanelet_end_m;

      for (size_t i = 0; i < route_ahead.ids.size(); ++i) {
        if (static_cast<int>(i) <= current_idx) {
          // Current or past lanelets -> negative distance to indicate "active/passed"
          cached_context_.upcoming_lanelet_distances_m.push_back(-1.0);
        } else {
          // Upcoming lanelets
          cached_context_.upcoming_lanelet_distances_m.push_back(dist_accumulator);

          // Add length of this lanelet to accumulator for the *next* one
          double len = 0.0;
          const auto & cl = route_ahead.lanelets[i].centerline;
          for (size_t k = 0; k + 1 < cl.size(); ++k) {
            double dx = cl[k + 1].x - cl[k].x;
            double dy = cl[k + 1].y - cl[k].y;
            len += std::sqrt(dx * dx + dy * dy);
          }
          dist_accumulator += len;
        }
      }
    } else {
      // current lanelet not in route (or no route)
      // fill with -1 to match size, though ids might be empty/irrelevant if off-route
      for (size_t i = 0; i < route_ahead.ids.size(); ++i) {
        cached_context_.upcoming_lanelet_distances_m.push_back(-1.0);
      }
    }
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  const LaneletHandler * lanelet_;
  EgoPoseHelper ego_pose_;
  double rate_hz_;
  double route_lookahead_m_;
  double route_priority_threshold_m_;
  double heading_search_radius_m_;

  rclcpp_lifecycle::LifecyclePublisher<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<int64_t> cached_lanelet_id_;
  lanelet_msgs::msg::CurrentLaneContext cached_context_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__PUBLISHERS__LANE_CONTEXT_PUBLISHER_HPP_
