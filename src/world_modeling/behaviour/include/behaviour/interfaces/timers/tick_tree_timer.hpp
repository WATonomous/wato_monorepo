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

#ifndef BEHAVIOUR__INTERFACES__TIMERS__TICK_TREE_TIMER_HPP_
#define BEHAVIOUR__INTERFACES__TIMERS__TICK_TREE_TIMER_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "behaviour/behaviour_tree.hpp"
#include "behaviour/interfaces/interface_base.hpp"

namespace behaviour::interfaces
{
class TickTreeTimer final : public InterfaceBase
{
public:
  TickTreeTimer(
    rclcpp_lifecycle::LifecycleNode * node, std::shared_ptr<BehaviourTree> tree, std::chrono::milliseconds period)
  : node_(node)
  , tree_(std::move(tree))
  , period_(period)
  {}

  void activate() override
  {
    if (timer_) {
      return;
    }

    timer_ = node_->create_wall_timer(period_, [this]() { tree_->tick(); });
  }

  void deactivate() override
  {
    timer_.reset();
  }

private:
  rclcpp_lifecycle::LifecycleNode * node_;
  std::shared_ptr<BehaviourTree> tree_;
  std::chrono::milliseconds period_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace behaviour::interfaces

#endif  // BEHAVIOUR__INTERFACES__TIMERS__TICK_TREE_TIMER_HPP_
