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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__EGO_STOPPED_FOR_DURATION_CONDITION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__EGO_STOPPED_FOR_DURATION_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace behaviour
{
/**
 * @class EgoStoppedForDurationCondition
 * @brief Returns SUCCESS only after ego has remained below the speed threshold for a duration.
 */
class EgoStoppedForDurationCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  EgoStoppedForDurationCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Odometry::SharedPtr>("ego_odom"),
      BT::InputPort<double>("threshold_velocity"),
      BT::InputPort<int>("duration_msec"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input");
    };

    auto ego_odom = ports::tryGetPtr<nav_msgs::msg::Odometry>(*this, "ego_odom");
    if (!ports::require(ego_odom, "ego_odom", missing_input_callback)) {
      resetStopTimer();
      return BT::NodeStatus::FAILURE;
    }

    auto threshold_velocity = ports::tryGet<double>(*this, "threshold_velocity");
    if (!ports::require(threshold_velocity, "threshold_velocity", missing_input_callback)) {
      resetStopTimer();
      return BT::NodeStatus::FAILURE;
    }

    auto duration_msec = ports::tryGet<int>(*this, "duration_msec");
    if (!ports::require(duration_msec, "duration_msec", missing_input_callback)) {
      resetStopTimer();
      return BT::NodeStatus::FAILURE;
    }

    const double speed = std::hypot(ego_odom->twist.twist.linear.x, ego_odom->twist.twist.linear.y);
    if (speed > *threshold_velocity) {
      resetStopTimer();
      return BT::NodeStatus::FAILURE;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!stop_start_time_) {
      stop_start_time_ = now;
      return BT::NodeStatus::FAILURE;
    }

    const auto stopped_for = std::chrono::duration_cast<std::chrono::milliseconds>(now - *stop_start_time_);
    if (stopped_for.count() < *duration_msec) {
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  void resetStopTimer()
  {
    stop_start_time_.reset();
  }

  std::optional<std::chrono::steady_clock::time_point> stop_start_time_;
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__EGO_STOPPED_FOR_DURATION_CONDITION_HPP_
