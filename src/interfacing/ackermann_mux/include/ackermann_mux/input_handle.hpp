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

#include <mutex>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ackermann_mux
{

/**
 * @struct InputConfig
 * @brief Configuration for a single input source to the Ackermann mux.
 */
struct InputConfig
{
  std::string name;  ///< Human-readable name for this input.
  std::string topic;  ///< ROS topic to subscribe to for Ackermann commands.
  int priority{0};  ///< Priority level (higher = more important).

  bool has_mask{false};  ///< Whether this input has an associated mask topic.
  std::string mask_topic;  ///< Topic for masking this input (Bool message).

  bool safety_gating{false};  ///< If true, triggers emergency stop on timeout.
};

/**
 * @class InputHandle
 * @brief Manages a single input source for the Ackermann mux.
 *
 * Handles subscription to an Ackermann command topic and optional mask topic.
 * Tracks the last received command, timestamp, and mask state for use by
 * the mux node during priority arbitration.
 */
class InputHandle
{
public:
  /**
   * @brief Constructs an InputHandle with the given configuration.
   * @param node Pointer to the parent ROS node (for creating subscriptions).
   * @param cfg Configuration specifying topic, priority, and mask settings.
   */
  InputHandle(rclcpp_lifecycle::LifecycleNode * node, const InputConfig & cfg);

  /**
   * @brief Returns the configuration for this input.
   * @return Const reference to the InputConfig.
   */
  const InputConfig & cfg() const
  {
    return cfg_;
  }

  // --- Queries used by the mux ---

  /**
   * @brief Returns the timestamp of the last received command.
   * @return ROS time of the most recent command message.
   */
  rclcpp::Time last_cmd_time() const;

  /**
   * @brief Checks if this input is eligible for mux selection.
   * @return True if a command has been received and input is not masked.
   */
  bool eligible_for_mux() const;

  /**
   * @brief Checks if safety gating has tripped due to command timeout.
   * @param now Current ROS time.
   * @param safety_threshold_sec Maximum allowed age (seconds) for a command.
   * @return True if safety gating is enabled and command is stale or missing.
   */
  bool safety_trip(const rclcpp::Time & now, double safety_threshold_sec) const;

  /**
   * @brief Returns the last received Ackermann command.
   * @return Copy of the most recent AckermannDriveStamped message.
   */
  ackermann_msgs::msg::AckermannDriveStamped last_cmd() const;

private:
  /**
   * @brief Callback for incoming Ackermann command messages.
   * @param msg The received AckermannDriveStamped message.
   */
  void on_cmd_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg);

  /**
   * @brief Callback for incoming mask messages.
   * @param msg The received Bool message (true = masked/disabled).
   */
  void on_mask_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  InputConfig cfg_;

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mask_;

  // Cached state
  mutable std::mutex mtx_;

  bool has_cmd_{false};
  ackermann_msgs::msg::AckermannDriveStamped last_cmd_{};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  bool mask_value_{false};
  bool has_mask_msg_{false};
  rclcpp::Time last_mask_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace ackermann_mux
