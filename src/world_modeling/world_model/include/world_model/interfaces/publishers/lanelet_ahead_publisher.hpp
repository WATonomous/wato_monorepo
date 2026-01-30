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

#ifndef WORLD_MODEL__INTERFACES__PUBLISHERS__LANELET_AHEAD_PUBLISHER_HPP_
#define WORLD_MODEL__INTERFACES__PUBLISHERS__LANELET_AHEAD_PUBLISHER_HPP_

#include <chrono>
#include <string>

#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Publishes all lanelets within a radius from ego at a fixed rate.
 *
 * Unlike RouteAheadPublisher which returns lanelets ON THE ACTIVE ROUTE,
 * this returns ALL lanelets within the configured radius for general awareness.
 */
class LaneletAheadPublisher : public InterfaceBase
{
public:
  LaneletAheadPublisher(
    rclcpp_lifecycle::LifecycleNode * node,
    const LaneletHandler * lanelet_handler,
    tf2_ros::Buffer * tf_buffer,
    const std::string & map_frame,
    const std::string & base_frame,
    double rate_hz,
    double radius_m)
  : node_(node)
  , lanelet_(lanelet_handler)
  , ego_pose_(tf_buffer, map_frame, base_frame)
  , rate_hz_(rate_hz)
  , radius_m_(radius_m)
  {
    pub_ = node_->create_publisher<lanelet_msgs::msg::LaneletAhead>("lanelet_ahead", 10);
  }

  void activate() override
  {
    pub_->on_activate();

    if (rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / rate_hz_);
      timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&LaneletAheadPublisher::publish, this));
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
  void publish()
  {
    if (!lanelet_->isMapLoaded()) {
      return;
    }

    auto ego_pose = ego_pose_.getEgoPose();
    if (!ego_pose.has_value()) {
      return;
    }

    // Extract heading from quaternion
    double heading_rad = tf2::getYaw(ego_pose->pose.orientation);

    auto lanelet_ahead = lanelet_->getLaneletAhead(ego_pose->pose.position, heading_rad, radius_m_);
    lanelet_ahead.header.stamp = node_->get_clock()->now();
    lanelet_ahead.header.frame_id = ego_pose_.mapFrame();

    pub_->publish(lanelet_ahead);
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  const LaneletHandler * lanelet_;
  EgoPoseHelper ego_pose_;
  double rate_hz_;
  double radius_m_;

  rclcpp_lifecycle::LifecyclePublisher<lanelet_msgs::msg::LaneletAhead>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__PUBLISHERS__LANELET_AHEAD_PUBLISHER_HPP_
