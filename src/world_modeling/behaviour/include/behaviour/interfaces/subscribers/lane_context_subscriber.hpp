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

#ifndef BEHAVIOUR__INTERFACES__SUBSCRIBERS__LANE_CONTEXT_SUBSCRIBER_HPP_
#define BEHAVIOUR__INTERFACES__SUBSCRIBERS__LANE_CONTEXT_SUBSCRIBER_HPP_

#include <memory>
#include <utility>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "behaviour/behaviour_tree.hpp"
#include "behaviour/interfaces/interface_base.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"

namespace behaviour::interfaces
{
class LaneContextSubscriber final : public InterfaceBase
{
public:
  LaneContextSubscriber(rclcpp_lifecycle::LifecycleNode * node, std::shared_ptr<BehaviourTree> tree)
  : node_(node)
  , tree_(std::move(tree))
  {}

  void activate() override
  {
    if (subscription_) {
      return;
    }

    subscription_ = node_->create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
      "current_lane_context",
      rclcpp::SystemDefaultsQoS(),
      [this](const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg) {
        tree_->updateBlackboard("current_lane_context", msg);
      });
  }

  void deactivate() override
  {
    subscription_.reset();
  }

private:
  rclcpp_lifecycle::LifecycleNode * node_;
  std::shared_ptr<BehaviourTree> tree_;
  rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr subscription_;
};
}  // namespace behaviour::interfaces

#endif  // BEHAVIOUR__INTERFACES__SUBSCRIBERS__LANE_CONTEXT_SUBSCRIBER_HPP_
