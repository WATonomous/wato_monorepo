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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__IS_AREA_OCCUPIED_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__IS_AREA_OCCUPIED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include "behaviour/nodes/logged_bt_node.hpp"

#include <iostream>
#include <string>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "world_model_msgs/msg/area_occupancy_info.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{
/**
 * @class IsAreaOccupiedCondition
 * @brief Returns SUCCESS when the requested occupancy area is currently occupied.
 */
class IsAreaOccupiedCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  IsAreaOccupiedCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<world_model_msgs::msg::AreaOccupancyInfo>>(
        "fetched_areas", "Latest area occupancy snapshot"),
      BT::InputPort<std::string>("area_name", "Area name to check"),
      BT::OutputPort<std::vector<world_model_msgs::msg::WorldObject>>("objects", "Objects found in the area"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input" );
    };

    auto fetched_areas = ports::tryGet<std::vector<world_model_msgs::msg::AreaOccupancyInfo>>(*this, "fetched_areas");
    if (!ports::require(fetched_areas, "fetched_areas", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto area_name = ports::tryGet<std::string>(*this, "area_name");
    if (!ports::require(area_name, "area_name", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput("objects", std::vector<world_model_msgs::msg::WorldObject>{});

    if (area_name->empty()) {
      RCLCPP_DEBUG_STREAM(logger(), "Empty area_name input" );
      return BT::NodeStatus::FAILURE;
    }

    const auto objects = area_occupancy_utils::getAreaObjects(*fetched_areas, *area_name);
    setOutput("objects", objects);

    if (area_occupancy_utils::isAreaOccupied(*fetched_areas, *area_name)) {
      RCLCPP_DEBUG_STREAM(logger(), "Occupied area found: " << *area_name );
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG_STREAM(logger(), "Area is clear: " << *area_name );
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__IS_AREA_OCCUPIED_CONDITION_HPP_
