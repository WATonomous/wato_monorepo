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
#include <std_msgs/msg/bool.hpp>

namespace ackermann_mux
{

struct InputConfig
{
  std::string name;
  std::string topic;
  int priority{0};

  bool has_mask{false};
  std::string mask_topic;

  bool safety_gating{false};
};

class InputHandle
{
public:
  InputHandle(rclcpp::Node * node, const InputConfig & cfg);

  const InputConfig & cfg() const
  {
    return cfg_;
  }

  // --- Queries used by the mux ---
  rclcpp::Time last_cmd_time() const;

  // eligible iff we have ever seen a cmd and not masked
  bool eligible_for_mux() const;

  // Safety gating trip: if enabled and (never seen cmd OR age > safety_threshold)
  bool safety_trip(const rclcpp::Time & now, double safety_threshold_sec) const;

  ackermann_msgs::msg::AckermannDriveStamped last_cmd() const;

private:
  void on_cmd_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg);
  void on_mask_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  rclcpp::Node * node_{nullptr};
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
