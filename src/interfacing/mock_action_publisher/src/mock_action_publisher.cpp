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

#include "mock_action_publisher/mock_action_publisher.hpp"

#include <chrono>
#include <memory>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

using std::chrono::milliseconds;
using std::chrono::nanoseconds;

namespace mock_action_publisher
{

MockActionPublisherNode::MockActionPublisherNode(const rclcpp::NodeOptions & options)
: Node(
    "mock_action_publisher",
    rclcpp::NodeOptions(options).allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
      true))
{
  // Parameters (with defaults if not provided)
  publish_rate_hz_ = this->get_parameter_or("publish_rate_hz", 20.0);
  speed_ = this->get_parameter_or("speed", 1.0);
  steering_angle_ = this->get_parameter_or("steering_angle", 1.0);

  if (publish_rate_hz_ <= 0.0) {
    publish_rate_hz_ = 20.0;
  }

  // Publishers
  ackermann_pub_ =
    this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/action/ackermann", rclcpp::QoS(10));
  idle_pub_ = this->create_publisher<std_msgs::msg::Bool>("/action/is_idle", rclcpp::QoS(10));

  // Service
  set_idle_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "/action/set_idle",
    std::bind(&MockActionPublisherNode::on_set_idle, this, std::placeholders::_1, std::placeholders::_2));

  // Timers
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  cmd_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&MockActionPublisherNode::on_timer, this));

  // publish idle state at 10 Hz so mux always has a fresh mask signal
  const auto idle_period = std::chrono::duration<double>(0.1);  // 10 Hz
  idle_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(idle_period),
    std::bind(&MockActionPublisherNode::on_idle_timer, this));

  RCLCPP_INFO(
    this->get_logger(),
    "MockActionPublisher started: rate=%.1fHz speed=%.2f steer=%.2f service=/action/set_idle",
    publish_rate_hz_,
    speed_,
    steering_angle_);
}

void MockActionPublisherNode::on_set_idle(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  {
    std::scoped_lock<std::mutex> lk(mtx_);
    is_idle_ = req->data;
  }

  res->success = true;
  res->message = req->data ? "is_idle set to true" : "is_idle set to false";

  RCLCPP_WARN(this->get_logger(), "Service /action/set_idle -> %s", req->data ? "true" : "false");
}

void MockActionPublisherNode::on_idle_timer()
{
  bool idle = false;
  {
    std::scoped_lock<std::mutex> lk(mtx_);
    idle = is_idle_;
  }

  std_msgs::msg::Bool msg;
  msg.data = idle;
  idle_pub_->publish(msg);
}

void MockActionPublisherNode::on_timer()
{
  bool idle = false;
  {
    std::scoped_lock<std::mutex> lk(mtx_);
    idle = is_idle_;
  }

  ackermann_msgs::msg::AckermannDriveStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";

  if (!idle) {
    cmd.drive.speed = speed_;
    cmd.drive.steering_angle = steering_angle_;
  } else {
    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;
  }

  ackermann_pub_->publish(cmd);
}

}  // namespace mock_action_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(mock_action_publisher::MockActionPublisherNode)
