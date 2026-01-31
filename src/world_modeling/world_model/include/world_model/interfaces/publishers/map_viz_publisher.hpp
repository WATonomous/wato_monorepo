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

#ifndef WORLD_MODEL__INTERFACES__PUBLISHERS__MAP_VIZ_PUBLISHER_HPP_
#define WORLD_MODEL__INTERFACES__PUBLISHERS__MAP_VIZ_PUBLISHER_HPP_

#include <chrono>
#include <string>

#include "lanelet_msgs/msg/map_visualization.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Publishes map visualization data at a fixed rate.
 *
 * Queries TF for ego pose and LaneletHandler for nearby lanelets
 * around the ego vehicle and publishes them for visualization.
 */
class MapVizPublisher : public InterfaceBase
{
public:
  MapVizPublisher(
    rclcpp_lifecycle::LifecycleNode * node,
    const LaneletHandler * lanelet_handler,
    tf2_ros::Buffer * tf_buffer)
  : node_(node)
  , lanelet_(lanelet_handler)
  , ego_pose_(tf_buffer,
              node->get_parameter("map_frame").as_string(),
              node->get_parameter("base_frame").as_string())
  {
    rate_hz_ = node_->declare_parameter<double>("map_viz_publish_rate_hz", 1.0);
    radius_m_ = node_->declare_parameter<double>("map_viz_radius_m", 100.0);

    pub_ = node_->create_publisher<lanelet_msgs::msg::MapVisualization>("map_visualization", 10);
  }

  void activate() override
  {
    pub_->on_activate();

    if (rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / rate_hz_);
      timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&MapVizPublisher::publish, this));
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
   * @brief Timer callback that publishes nearby lanelet geometry for map visualization.
   *
   * Looks up ego position via TF and queries LaneletHandler for all lanelets
   * within the configured radius, converting each to a message with full
   * boundary, centerline, and regulatory element data.
   */
  void publish()
  {
    if (!lanelet_->isMapLoaded()) {
      return;
    }

    auto center = ego_pose_.getEgoPoint();
    if (!center.has_value()) {
      return;
    }

    lanelet_msgs::msg::MapVisualization msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = ego_pose_.mapFrame();

    auto nearby = lanelet_->getLaneletsInRadius(*center, radius_m_);
    for (const auto & ll : nearby) {
      msg.lanelets.push_back(lanelet_->toLaneletMsg(ll));
    }

    pub_->publish(msg);
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  const LaneletHandler * lanelet_;
  EgoPoseHelper ego_pose_;
  double rate_hz_;
  double radius_m_;

  rclcpp_lifecycle::LifecyclePublisher<lanelet_msgs::msg::MapVisualization>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__PUBLISHERS__MAP_VIZ_PUBLISHER_HPP_
