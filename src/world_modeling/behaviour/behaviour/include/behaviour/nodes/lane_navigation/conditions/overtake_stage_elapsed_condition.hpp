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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__OVERTAKE_STAGE_ELAPSED_CONDITION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__OVERTAKE_STAGE_ELAPSED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include "behaviour/nodes/bt_logger_base.hpp"

#include <chrono>
#include <optional>
#include <string>

#include "behaviour/utils/ports.hpp"
#include "behaviour/utils/types.hpp"

namespace behaviour
{
class OvertakeStageElapsedCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  OvertakeStageElapsedCondition(
    const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config), BTLoggerBase(logger)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::OvertakeStage>("stage"),
      BT::InputPort<types::OvertakeStage>("expected"),
      BT::InputPort<int>("duration_msec"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port_name);
    };

    auto stage = ports::tryGet<types::OvertakeStage>(*this, "stage");
    if (!ports::require(stage, "stage", missing_input_callback)) {
      resetTimer();
      return BT::NodeStatus::FAILURE;
    }

    auto expected = ports::tryGet<types::OvertakeStage>(*this, "expected");
    if (!ports::require(expected, "expected", missing_input_callback)) {
      resetTimer();
      return BT::NodeStatus::FAILURE;
    }

    auto duration_msec = ports::tryGet<int>(*this, "duration_msec");
    if (!ports::require(duration_msec, "duration_msec", missing_input_callback)) {
      resetTimer();
      return BT::NodeStatus::FAILURE;
    }

    if (*stage != *expected) {
      resetTimer();
      return BT::NodeStatus::FAILURE;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!stage_start_time_) {
      stage_start_time_ = now;
      return BT::NodeStatus::FAILURE;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - *stage_start_time_);
    if (elapsed.count() < *duration_msec) {
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  void resetTimer()
  {
    stage_start_time_.reset();
  }

  std::optional<std::chrono::steady_clock::time_point> stage_start_time_;
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__OVERTAKE_STAGE_ELAPSED_CONDITION_HPP_
