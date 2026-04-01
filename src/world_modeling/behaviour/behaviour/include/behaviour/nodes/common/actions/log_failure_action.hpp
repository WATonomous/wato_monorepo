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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__LOG_FAILURE_ACTION_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__LOG_FAILURE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"

namespace behaviour
{
/**
 * @class LogFailureAction
 * @brief Logs a failure message and returns FAILURE.
 */
class LogFailureAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  LogFailureAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message"), BT::InputPort<std::string>("error_message")};
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_ERROR(logger(), "[%s] Missing input port: %s", name().c_str(), port_name);
    };

    auto message = ports::tryGet<std::string>(*this, "message");
    if (!ports::require(message, "message", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto error_message = ports::tryGet<std::string>(*this, "error_message");
    if (error_message && !error_message->empty()) {
      RCLCPP_ERROR(logger(), "[%s] %s: %s", name().c_str(), message->c_str(), error_message->c_str());
    } else {
      RCLCPP_ERROR(logger(), "[%s] %s", name().c_str(), message->c_str());
    }

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__LOG_FAILURE_ACTION_HPP_
