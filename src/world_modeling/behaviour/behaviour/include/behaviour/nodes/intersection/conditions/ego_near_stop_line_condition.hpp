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

#ifndef BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__EGO_NEAR_STOP_LINE_CONDITION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__EGO_NEAR_STOP_LINE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <cmath>
#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace behaviour
{
/**
 * @class EgoNearStopLineCondition
 * @brief Returns SUCCESS when ego is within a distance threshold of the stop line.
 */
class EgoNearStopLineCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  EgoNearStopLineCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Odometry::SharedPtr>("ego_odom"),
      BT::InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("stop_line_pose"),
      BT::InputPort<double>("threshold_m"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port_name);
    };

    auto ego_odom = ports::tryGetPtr<nav_msgs::msg::Odometry>(*this, "ego_odom");
    if (!ports::require(ego_odom, "ego_odom", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto stop_line_pose = ports::tryGetPtr<geometry_msgs::msg::PoseStamped>(*this, "stop_line_pose");
    if (!ports::require(stop_line_pose, "stop_line_pose", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto threshold_m = ports::tryGet<double>(*this, "threshold_m");
    if (!ports::require(threshold_m, "threshold_m", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const double dx = stop_line_pose->pose.position.x - ego_odom->pose.pose.position.x;
    const double dy = stop_line_pose->pose.position.y - ego_odom->pose.pose.position.y;
    const double distance_m = std::hypot(dx, dy);

    if (distance_m <= *threshold_m) {
      RCLCPP_DEBUG_STREAM(logger(), "ego_near_stop_line distance_m=" << distance_m << " threshold_m=" << *threshold_m);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG_STREAM(logger(), "ego_far_from_stop_line distance_m=" << distance_m << " threshold_m=" << *threshold_m);
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__EGO_NEAR_STOP_LINE_CONDITION_HPP_
