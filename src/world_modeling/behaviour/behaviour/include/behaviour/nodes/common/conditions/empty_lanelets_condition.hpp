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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__EMPTY_LANELETS_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__EMPTY_LANELETS_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/lanelet.hpp"
#include "behaviour/utils/ports.hpp"
#include "behaviour/utils/world_objects.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{
class EmptyLaneletsCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  EmptyLaneletsCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<int64_t>>("lanelet_ids"),
      BT::InputPort<std::vector<world_model_msgs::msg::WorldObject>>("objects"),
      BT::InputPort<std::size_t>("hypothesis_index"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port_name);
    };

    auto lanelet_ids = ports::tryGet<std::vector<int64_t>>(*this, "lanelet_ids");
    if (!ports::require(lanelet_ids, "lanelet_ids", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    if (lanelet_ids->empty()) {
      RCLCPP_DEBUG_STREAM(logger(), "empty_lanelet_ids");
      return BT::NodeStatus::FAILURE;
    }

    auto objects = ports::tryGet<std::vector<world_model_msgs::msg::WorldObject>>(*this, "objects");
    if (!ports::require(objects, "objects", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto hypothesis_index = ports::tryGet<std::size_t>(*this, "hypothesis_index");
    if (!ports::require(hypothesis_index, "hypothesis_index", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    for (const auto & object : *objects) {
      const auto object_lanelet_id = object.lanelet_ahead.current_lanelet_id;
      if (!utils::lanelet::containsLaneletId(*lanelet_ids, object_lanelet_id)) {
        continue;
      }

      if (!utils::world_objects::isRoadUser(object, *hypothesis_index)) {
        continue;
      }

      RCLCPP_DEBUG_STREAM(
        logger(), "lanelet_occupied lanelet_id=" << object_lanelet_id << " object_id=" << object.detection.id);
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__EMPTY_LANELETS_CONDITION_HPP_
