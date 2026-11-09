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
#include <mutex>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>

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

  /**
   * @brief Configures the node, including parameters and internal state.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates the node, enabling publishers and timers.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates the node, disabling publishers and timers.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleans up the node, releasing resources.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shuts down the node.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
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

  /**
   * @brief Callback for the fault-collector-driven emergency stop signal.
   * @param msg Latched Bool published by fault_collector's emergency_stop topic.
   */
  void fault_estop_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  /**
   * @brief Determines whether the fault-driven emergency stop is currently asserted.
   *
   * This is a hard override on top of priority arbitration and safety gating: if it
   * returns true, the mux publishes emergency (or a bypass input's command -- see
   * ackerman_cmd_callback()) instead of running normal arbitration. Returns false when
   * `fault_estop.enabled` is false. When enabled, treats a signal that has gone stale for
   * longer than `fault_estop.timeout_s` (if > 0) as asserted -- fail-safe on losing the
   * fault collector.
   *
   * @param now Current ROS time, used to check staleness.
   * @return True if the fault-driven emergency stop is currently asserted.
   */
  bool is_fault_estop_asserted(const rclcpp::Time & now) const;

  /**
   * @brief Selects the highest-priority eligible input that is allowed to bypass the fault
   * emergency stop (see `fault_estop.bypass_inputs`), so the operator is never locked out.
   * @return The winning bypass input, or nullptr if none is currently eligible.
   */
  std::shared_ptr<InputHandle> select_fault_estop_bypass_input() const;

  // Params
  double safety_threshold_{};  ///< Max age (seconds) before a command is considered stale.
  double publish_rate_hz_{};  ///< Rate at which to publish output commands.
  ackermann_msgs::msg::AckermannDriveStamped emergency_{};  ///< Emergency stop command.

  bool fault_estop_enabled_{false};  ///< Whether to subscribe to a fault-collector emergency stop signal.
  std::string fault_estop_topic_;  ///< Topic for the fault-collector emergency stop signal.
  double fault_estop_timeout_s_{0.0};  ///< Staleness timeout (s); 0.0 disables staleness checking.
  std::vector<std::string> fault_estop_bypass_inputs_;  ///< Input names allowed to bypass the fault estop.

  // IO
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_out_;  ///< Output publisher.
  rclcpp::TimerBase::SharedPtr timer_;  ///< Periodic timer for command selection.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fault_estop_sub_;  ///< Fault-collector emergency stop input.

  // Input handles --dynamic array
  std::vector<std::shared_ptr<InputHandle>> inputs_;  ///< List of input handles.

  // Cached fault-estop state. The node is single-threaded via the timer, but this is
  // guarded like InputHandle's cached state for consistency and safety.
  mutable std::mutex fault_estop_mtx_;
  bool fault_estop_value_{false};  ///< Most recently received emergency stop value.
  bool has_fault_estop_msg_{false};  ///< Whether any emergency stop message has been received yet.
  rclcpp::Time last_fault_estop_time_{0, 0, RCL_ROS_TIME};  ///< Time of the last emergency stop message.
};

}  // namespace ackermann_mux
