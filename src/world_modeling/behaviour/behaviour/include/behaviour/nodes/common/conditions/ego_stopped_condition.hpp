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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace behaviour
{
/** @brief Check stopped speed using the median of distinct, fresh odometry samples. */
class EgoStoppedCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  EgoStoppedCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {
    // BehaviourTree already seeds the root blackboard with its ROS node. Use its clock
    // so message age is measured correctly with use_sim_time, without changing BT ports.
    rclcpp::Node::SharedPtr node;
    if (config.blackboard && config.blackboard->get("@node", node) && node) {
      clock_ = node->get_clock();
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Odometry::SharedPtr>("ego_odom"),
      BT::InputPort<double>("threshold_velocity"),
      BT::InputPort<int>("window_size", 3, "Number of distinct velocity samples for the median"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input");
    };
    const auto ego_odom = ports::tryGetPtr<nav_msgs::msg::Odometry>(*this, "ego_odom");
    const auto threshold = ports::tryGet<double>(*this, "threshold_velocity");
    if (
      !ports::require(ego_odom, "ego_odom", missing_input_callback) ||
      !ports::require(threshold, "threshold_velocity", missing_input_callback) || !clock_)
    {
      velocity_history_.clear();
      return BT::NodeStatus::FAILURE;
    }

    const auto & stamp = ego_odom->header.stamp;
    const int64_t stamp_ns = static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
    const auto ros_now_ns = clock_->now().nanoseconds();
    const double speed = std::hypot(ego_odom->twist.twist.linear.x, ego_odom->twist.twist.linear.y);
    if (
      stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U || stamp_ns <= 0 || ros_now_ns < stamp_ns ||
      ros_now_ns - stamp_ns > kMaxOdomAgeNs || !std::isfinite(speed) || !std::isfinite(*threshold) || *threshold < 0.0)
    {
      velocity_history_.clear();
      return BT::NodeStatus::FAILURE;
    }

    const auto now = std::chrono::steady_clock::now();
    if (last_stamp_ns_ != 0 && stamp_ns < last_stamp_ns_) {
      // Discard a reversed sample, then allow recovery on the new clock timeline.
      velocity_history_.clear();
      last_stamp_ns_ = stamp_ns;
      last_sample_time_ = now;
      return BT::NodeStatus::FAILURE;
    }
    if (
      last_stamp_ns_ != 0 &&
      (stamp_ns - last_stamp_ns_ > kMaxOdomAgeNs || now - last_sample_time_ > std::chrono::nanoseconds(kMaxOdomAgeNs)))
    {
      velocity_history_.clear();
    }
    if (stamp_ns != last_stamp_ns_) {
      velocity_history_.push_back(speed);
      last_stamp_ns_ = stamp_ns;
      last_sample_time_ = now;
    }

    const auto window_size =
      static_cast<std::size_t>(std::max(1, ports::tryGet<int>(*this, "window_size").value_or(3)));
    while (velocity_history_.size() > window_size) {
      velocity_history_.pop_front();
    }
    if (velocity_history_.size() < window_size) {
      return BT::NodeStatus::FAILURE;
    }

    std::vector<double> sorted(velocity_history_.begin(), velocity_history_.end());
    std::sort(sorted.begin(), sorted.end());
    // For even windows, use the upper median so at least a strict majority must be low.
    const double median = sorted[sorted.size() / 2];
    RCLCPP_DEBUG_STREAM(logger(), "median speed " << median << ", threshold_velocity " << *threshold);
    return median <= *threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  static constexpr int64_t kMaxOdomAgeNs = 500'000'000;
  rclcpp::Clock::SharedPtr clock_;
  std::deque<double> velocity_history_;
  int64_t last_stamp_ns_{0};
  std::chrono::steady_clock::time_point last_sample_time_{};
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__EGO_STOPPED_CONDITION_HPP_

