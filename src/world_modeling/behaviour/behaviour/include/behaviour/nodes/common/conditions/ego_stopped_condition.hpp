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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__EGO_STOPPED_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__EGO_STOPPED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace behaviour
{
/**
 * @class EgoStoppedCondition
 * @brief ConditionNode to check whether ego speed is below a threshold using a moving average window.
 */

class EgoStoppedCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  EgoStoppedCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Odometry::SharedPtr>("ego_odom"),
      BT::InputPort<double>("threshold_velocity"),
      BT::InputPort<int>("window_size", 5, "Number of velocity samples to average"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input");
    };

    auto ego_odom = ports::tryGetPtr<nav_msgs::msg::Odometry>(*this, "ego_odom");
    if (!ports::require(ego_odom, "ego_odom", missing_input_callback)) {
      velocity_history_.clear();
      return BT::NodeStatus::FAILURE;
    }
    auto threshold_velocity = ports::tryGet<double>(*this, "threshold_velocity");
    if (!ports::require(threshold_velocity, "threshold_velocity", missing_input_callback)) {
      velocity_history_.clear();
      return BT::NodeStatus::FAILURE;
    }

    int window_size = ports::tryGet<int>(*this, "window_size").value_or(5);
    if (window_size <= 0) {
      window_size = 1;
    }

    const double v = std::hypot(ego_odom->twist.twist.linear.x, ego_odom->twist.twist.linear.y);

    velocity_history_.push_back(v);
    while (velocity_history_.size() > static_cast<size_t>(window_size)) {
      velocity_history_.pop_front();
    }

    double sum = 0.0;
    for (double speed : velocity_history_) {
      sum += speed;
    }
    const double avg_v = sum / static_cast<double>(velocity_history_.size());
    const double thresh = *threshold_velocity;

    if (avg_v <= thresh) {
      RCLCPP_DEBUG_STREAM(
        logger(), "avg speed " << avg_v << " (inst: " << v << ", samples: " << velocity_history_.size()
                               << ") <= threshold_velocity " << thresh);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG_STREAM(
      logger(), "avg speed " << avg_v << " (inst: " << v << ", samples: " << velocity_history_.size()
                             << ") > threshold_velocity " << thresh);
    return BT::NodeStatus::FAILURE;
  }

private:
  std::deque<double> velocity_history_;
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__EGO_STOPPED_CONDITION_HPP_
