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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_REACHED_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_REACHED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/utils.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace behaviour
{
/**
   * @class GoalReachedCondition
   * @brief ConditionNode to check whether ego is within goal distance threshold.
   */
class GoalReachedCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  GoalReachedCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Odometry::SharedPtr>("ego_odom"),
      BT::InputPort<geometry_msgs::msg::PointStamped::SharedPtr>("goal_point"),
      BT::InputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("goal_lanelet"),
      BT::InputPort<std::string>("mode"),
      BT::InputPort<double>("threshold_m")};
  };

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input");
    };

    auto ego_odom = ports::tryGetPtr<nav_msgs::msg::Odometry>(*this, "ego_odom");
    if (!ports::require(ego_odom, "ego_odom", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    std::string mode = ports::tryGet<std::string>(*this, "mode").value_or("lanelet");
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) { return std::tolower(c); });

    auto threshold_m = ports::tryGet<double>(*this, "threshold_m");
    if (!ports::require(threshold_m, "threshold_m", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const auto & cp = ego_odom->pose.pose.position;
    double target_x = 0.0;
    double target_y = 0.0;
    std::string target_desc;

    if (mode == "lanelet") {
      auto goal_lanelet = ports::tryGetPtr<lanelet_msgs::msg::Lanelet>(*this, "goal_lanelet");
      if (!ports::require(goal_lanelet, "goal_lanelet", missing_input_callback)) {
        return BT::NodeStatus::FAILURE;
      }
      if (goal_lanelet->centerline.empty()) {
        RCLCPP_DEBUG_STREAM(logger(), "goal_lanelet centerline is empty");
        return BT::NodeStatus::FAILURE;
      }

      const auto & lanelet_end_point = goal_lanelet->centerline.back();
      target_x = lanelet_end_point.x;
      target_y = lanelet_end_point.y;
      target_desc = "goal_lanelet_id=" + std::to_string(goal_lanelet->id);
    } else if (mode == "point") {
      auto goal_point = ports::tryGetPtr<geometry_msgs::msg::PointStamped>(*this, "goal_point");
      if (!ports::require(goal_point, "goal_point", missing_input_callback)) {
        return BT::NodeStatus::FAILURE;
      }

      target_x = goal_point->point.x;
      target_y = goal_point->point.y;
      target_desc = "goal_point=(" + std::to_string(target_x) + ", " + std::to_string(target_y) + ")";
    } else {
      RCLCPP_ERROR_STREAM(logger(), "Invalid goal reached mode '" << mode << "'. Expected 'lanelet' or 'point'");
      return BT::NodeStatus::FAILURE;
    }
    const double threshold = *threshold_m;

    // Calculate 2D distance
    const double dx = target_x - cp.x;
    const double dy = target_y - cp.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    // Check if within threshold
    if (distance <= threshold) {
      RCLCPP_DEBUG_STREAM(
        logger(), "Distance " << distance << " is within threshold " << threshold << " (mode=" << mode << ", "
                               << target_desc << ")");
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG_STREAM(
      logger(), "Distance " << distance << " is outside threshold " << threshold << " (mode=" << mode << ", "
                             << target_desc << ")");
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour
#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_REACHED_CONDITION_HPP_
