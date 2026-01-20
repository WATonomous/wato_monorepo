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

#include "ackermann_mux/input_handle.hpp"

namespace ackermann_mux
{

InputHandle::InputHandle(rclcpp::Node * node, const InputConfig & cfg)
: node_(node)
, cfg_(cfg)
{
  sub_cmd_ = node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    cfg_.topic, rclcpp::SystemDefaultsQoS(), std::bind(&InputHandle::on_cmd_callback, this, std::placeholders::_1));

  if (cfg_.has_mask) {
    if (cfg_.mask_topic.empty()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Input '%s' has_mask=true but mask_topic is empty. Treating as unmasked.",
        cfg_.name.c_str());
    } else {
      sub_mask_ = node_->create_subscription<std_msgs::msg::Bool>(
        cfg_.mask_topic,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&InputHandle::on_mask_callback, this, std::placeholders::_1));
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured input '%s': topic=%s priority=%d has_mask=%s mask_topic=%s safety_gating=%s",
    cfg_.name.c_str(),
    cfg_.topic.c_str(),
    cfg_.priority,
    cfg_.has_mask ? "true" : "false",
    cfg_.mask_topic.empty() ? "(none)" : cfg_.mask_topic.c_str(),
    cfg_.safety_gating ? "true" : "false");
}

void InputHandle::on_cmd_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg)
{
  std::scoped_lock<std::mutex> lk(mtx_);
  last_cmd_ = *msg;
  last_cmd_time_ = node_->now();
  has_cmd_ = true;
}

void InputHandle::on_mask_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  std::scoped_lock<std::mutex> lk(mtx_);
  mask_value_ = msg->data;
  has_mask_msg_ = true;
  last_mask_time_ = node_->now();
}

rclcpp::Time InputHandle::last_cmd_time() const
{
  std::scoped_lock<std::mutex> lk(mtx_);
  return last_cmd_time_;
}

bool InputHandle::eligible_for_mux() const
{
  std::scoped_lock<std::mutex> lk(mtx_);
  if (!has_cmd_) {
    return false;
  }
  if (cfg_.has_mask && !cfg_.mask_topic.empty() && mask_value_) {
    return false;
  }

  return true;
}

bool InputHandle::safety_trip(const rclcpp::Time & now, double safety_threshold_sec) const
{
  if (!cfg_.safety_gating) {
    return false;
  }

  std::scoped_lock<std::mutex> lk(mtx_);

  if (!has_cmd_) {
    return true;
  }

  const double age_sec = (now - last_cmd_time_).seconds();
  return age_sec > safety_threshold_sec;
}

ackermann_msgs::msg::AckermannDriveStamped InputHandle::last_cmd() const
{
  std::scoped_lock<std::mutex> lk(mtx_);
  return last_cmd_;
}

}  // namespace ackermann_mux
