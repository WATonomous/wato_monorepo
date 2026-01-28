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

#ifndef BEHAVIOUR__NODES__CONDITIONS__COMPARATOR_CONDITION_HPP_
#define BEHAVIOUR__NODES__CONDITIONS__COMPARATOR_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class ComparatorCondition
   * @brief A generic condition node to compare two numeric values.
   *
   * Inputs:
   *   - left: The left-hand value
   *   - operator: The comparison operator ("==", "!=", ">", "<", ">=", "<=")
   *   - right: The right-hand value
   */
class ComparatorCondition : public BT::ConditionNode
{
public:
  ComparatorCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("left", "Left-hand side value"),
      BT::InputPort<std::string>("operator", "Comparison operator (==, !=, >, <, >=, <=)"),
      BT::InputPort<double>("right", "Right-hand side value")};
  }

  BT::NodeStatus tick() override
  {
    double left;
    double right;
    std::string op;

    try {
      left = ports::get<double>(*this, "left");
      right = ports::get<double>(*this, "right");
      op = ports::get<std::string>(*this, "operator");
    } catch (const BT::RuntimeError & e) {
      return BT::NodeStatus::FAILURE;
    }

    if (op == "==") return (left == right) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    if (op == "!=") return (left != right) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    if (op == ">") return (left > right) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    if (op == "<") return (left < right) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    if (op == ">=") return (left >= right) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    if (op == "<=") return (left <= right) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

    RCLCPP_ERROR(rclcpp::get_logger("ComparatorCondition"), "Invalid operator: %s", op.c_str());
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__CONDITIONS__COMPARATOR_CONDITION_HPP_
