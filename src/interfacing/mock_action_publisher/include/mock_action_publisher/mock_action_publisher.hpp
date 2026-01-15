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

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace mock_action_publisher
{

class MockActionPublisherNode : public rclcpp::Node
{
public:
  explicit MockActionPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_timer();
  void on_idle_timer();

  void on_set_idle(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  // Params
  double publish_rate_hz_{20.0};
  double speed_{1.0};
  double steering_angle_{1.0};

  // State
  mutable std::mutex mtx_;
  bool is_idle_{false};

  // Pub/Sub/Service
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr idle_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_idle_srv_;

  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr idle_timer_;
};

}  // namespace mock_action_publisher
