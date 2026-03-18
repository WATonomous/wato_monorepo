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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__EGO_ON_LANELET_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__EGO_ON_LANELET_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include "behaviour/nodes/logged_bt_node.hpp"

#include <iostream>
#include <string>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"

namespace behaviour
{
/**
 * @class EgoOnLaneletCondition
 * @brief Returns SUCCESS when the ego vehicle is currently on the provided lanelet.
 */
class EgoOnLaneletCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  EgoOnLaneletCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::InputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("lanelet"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input" );
    };

    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!ports::require(lane_ctx, "lane_ctx", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto lanelet = ports::tryGetPtr<lanelet_msgs::msg::Lanelet>(*this, "lanelet");
    if (!ports::require(lanelet, "lanelet", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const bool ego_on_lanelet = lane_ctx->current_lanelet.id == lanelet->id;
    RCLCPP_DEBUG_STREAM(logger(), "current=" << lane_ctx->current_lanelet.id << ", target=" << lanelet->id
              << ", result=" << (ego_on_lanelet ? "true" : "false") );
    return ego_on_lanelet ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__EGO_ON_LANELET_CONDITION_HPP_
