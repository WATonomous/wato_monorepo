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

#ifndef WORLD_MODEL__INTERFACES__PUBLISHERS__ROUTE_AHEAD_PUBLISHER_HPP_
#define WORLD_MODEL__INTERFACES__PUBLISHERS__ROUTE_AHEAD_PUBLISHER_HPP_

#include <chrono>
#include <string>

#include "lanelet_msgs/msg/route_ahead.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Publishes route-ahead lanelets at a fixed rate.
 *
 * Queries TF for ego pose and LaneletHandler for route lanelets ahead
 * within the configured lookahead distance. Enables BFS via successor_ids.
 */
class RouteAheadPublisher : public InterfaceBase
{
public:
  RouteAheadPublisher(
    rclcpp_lifecycle::LifecycleNode * node,
    const LaneletHandler * lanelet_handler,
    tf2_ros::Buffer * tf_buffer)
  : node_(node)
  , lanelet_(lanelet_handler)
  , ego_pose_(tf_buffer,
              node->get_parameter("map_frame").as_string(),
              node->get_parameter("base_frame").as_string())
  {
    rate_hz_ = node_->declare_parameter<double>("route_ahead_publish_rate_hz", 10.0);
    lookahead_m_ = node_->declare_parameter<double>("route_ahead_lookahead_m", 100.0);

    pub_ = node_->create_publisher<lanelet_msgs::msg::RouteAhead>("route_ahead", 10);
  }

  void activate() override
  {
    pub_->on_activate();

    if (rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / rate_hz_);
      timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&RouteAheadPublisher::publish, this));
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
   * @brief Timer callback that publishes lanelets along the active route within lookahead.
   *
   * Looks up ego position via TF and queries LaneletHandler for route lanelets
   * ahead of ego within the configured lookahead distance. Publishes an empty
   * message with has_active_route=false if no route is set.
   */
  void publish()
  {
    if (!lanelet_->isMapLoaded()) {
      return;
    }

    auto ego_point = ego_pose_.getEgoPoint();
    if (!ego_point.has_value()) {
      return;
    }

    auto route_ahead = lanelet_->getRouteAhead(*ego_point, lookahead_m_);
    route_ahead.header.stamp = node_->get_clock()->now();
    route_ahead.header.frame_id = ego_pose_.mapFrame();

    pub_->publish(route_ahead);
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  const LaneletHandler * lanelet_;
  EgoPoseHelper ego_pose_;
  double rate_hz_;
  double lookahead_m_;

  rclcpp_lifecycle::LifecyclePublisher<lanelet_msgs::msg::RouteAhead>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__PUBLISHERS__ROUTE_AHEAD_PUBLISHER_HPP_
