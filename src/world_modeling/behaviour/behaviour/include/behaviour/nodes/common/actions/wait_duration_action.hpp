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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__WAIT_DURATION_ACTION_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__WAIT_DURATION_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <optional>
#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"

namespace behaviour
{

class WaitDurationAction : public BT::StatefulActionNode, protected BTLoggerBase
{
public:
  WaitDurationAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::StatefulActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("duration_msec"),
    };
  }

  BT::NodeStatus onStart() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port_name);
    };

    auto duration_msec = ports::tryGet<int>(*this, "duration_msec");
    if (!ports::require(duration_msec, "duration_msec", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    if (*duration_msec <= 0) {
      return BT::NodeStatus::SUCCESS;
    }

    deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(*duration_msec);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!deadline_) {
      return BT::NodeStatus::FAILURE;
    }

    if (std::chrono::steady_clock::now() >= *deadline_) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    deadline_.reset();
  }

private:
  std::optional<std::chrono::steady_clock::time_point> deadline_;
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__WAIT_DURATION_ACTION_HPP_
