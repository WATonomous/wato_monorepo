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

#ifndef BEHAVIOUR__INTERFACES__TIMERS__EGO_POSE_TIMER_HPP_
#define BEHAVIOUR__INTERFACES__TIMERS__EGO_POSE_TIMER_HPP_

#include <tf2_ros/buffer.h>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviour/behaviour_tree.hpp"
#include "behaviour/interfaces/interface_base.hpp"

namespace behaviour::interfaces
{
class EgoPoseTimer final : public InterfaceBase
{
public:
  EgoPoseTimer(
    rclcpp_lifecycle::LifecycleNode * node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::string map_frame,
    std::string base_frame,
    std::shared_ptr<BehaviourTree> tree,
    std::chrono::milliseconds period)
  : node_(node)
  , tf_buffer_(std::move(tf_buffer))
  , map_frame_(std::move(map_frame))
  , base_frame_(std::move(base_frame))
  , tree_(std::move(tree))
  , period_(period)
  {}

  void activate() override
  {
    if (timer_) {
      return;
    }

    timer_ = node_->create_wall_timer(period_, [this]() { update(); });
  }

  void deactivate() override
  {
    timer_.reset();
  }

private:
  void update()
  {
    try {
      auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);

      auto point = std::make_shared<geometry_msgs::msg::Point>();
      point->x = transform.transform.translation.x;
      point->y = transform.transform.translation.y;
      point->z = transform.transform.translation.z;

      tree_->updateBlackboard("current_point", point);
    } catch (const tf2::TransformException & e) {
      RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        5000,
        "Waiting for %s -> %s transform: %s",
        map_frame_.c_str(),
        base_frame_.c_str(),
        e.what());
    }
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string map_frame_;
  std::string base_frame_;
  std::shared_ptr<BehaviourTree> tree_;
  std::chrono::milliseconds period_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace behaviour::interfaces

#endif  // BEHAVIOUR__INTERFACES__TIMERS__EGO_POSE_TIMER_HPP_
