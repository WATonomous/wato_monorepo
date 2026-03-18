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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__WORLD_OBJECTS_CONTAINS_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__WORLD_OBJECTS_CONTAINS_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>
#include <vector>

#include "behaviour/nodes/logged_bt_node.hpp"
#include "behaviour/utils/utils.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{
/**
 * @class WorldObjectsContainsCondition
 * @brief Returns SUCCESS when the input object list contains at least one object of the requested type.
 */
class WorldObjectsContainsCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  WorldObjectsContainsCondition(
    const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<world_model_msgs::msg::WorldObject>>("world_objects", "Objects to inspect"),
      BT::InputPort<std::string>("object_type", "Requested object type"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input");
    };

    auto objects = ports::tryGet<std::vector<world_model_msgs::msg::WorldObject>>(*this, "world_objects");
    if (!ports::require(objects, "world_objects", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto object_type = ports::tryGet<std::string>(*this, "object_type");
    if (!ports::require(object_type, "object_type", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    if (object_type->empty()) {
      RCLCPP_DEBUG_STREAM(logger(), "Empty object_type input");
      return BT::NodeStatus::FAILURE;
    }

    const bool contains = world_objects::containsType(*objects, *object_type);
    RCLCPP_DEBUG_STREAM(
      logger(), "Checking " << objects->size() << " object(s) for type '" << *object_type
                            << "': " << (contains ? "found" : "not found"));

    return contains ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__WORLD_OBJECTS_CONTAINS_CONDITION_HPP_
