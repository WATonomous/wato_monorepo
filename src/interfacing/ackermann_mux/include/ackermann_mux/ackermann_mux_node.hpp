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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "ackermann_mux/input_handle.hpp"

namespace ackermann_mux
{

/**
 * @class AckermannMuxNode
 * @brief ROS 2 node that multiplexes multiple Ackermann drive command sources.
 *
 * This node subscribes to multiple Ackermann drive command topics and outputs
 * a single command based on priority and safety gating rules. It supports
 * masking individual inputs and enforces safety thresholds for command timeouts.
 */
class AckermannMuxNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructs the AckermannMuxNode.
   * @param options Node options for configuring the ROS 2 node.
   */
  explicit AckermannMuxNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief Reads input configuration from parameters and creates InputHandle objects.
   *
   * Parses the 'inputs' parameter array and instantiates an InputHandle for each
   * configured input source with its topic, priority, and optional mask.
   */
  void build_inputs_from_params();

  /**
   * @brief Timer callback that selects and publishes the highest-priority command.
   *
   * Iterates through all inputs, checks eligibility and safety conditions,
   * and publishes either the winning command or an emergency stop command.
   */
  void ackerman_cmd_callback();

  // Params
  double safety_threshold_{};  ///< Max age (seconds) before a command is considered stale.
  double publish_rate_hz_{};  ///< Rate at which to publish output commands.
  ackermann_msgs::msg::AckermannDriveStamped emergency_{};  ///< Emergency stop command.

  // IO
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_out_;  ///< Output publisher.
  rclcpp::TimerBase::SharedPtr timer_;  ///< Periodic timer for command selection.

  // Input handles --dynamic array
  std::vector<std::shared_ptr<InputHandle>> inputs_;  ///< List of input handles.
};

}  // namespace ackermann_mux
