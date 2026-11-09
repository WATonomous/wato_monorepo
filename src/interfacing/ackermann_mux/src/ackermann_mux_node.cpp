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

#include "ackermann_mux/ackermann_mux_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>

using std::chrono::nanoseconds;

namespace ackermann_mux
{

AckermannMuxNode::AckermannMuxNode(const rclcpp::NodeOptions & options)
: LifecycleNode(
    "ackermann_mux",
    rclcpp::NodeOptions(options).allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
      true))
{
  RCLCPP_INFO(this->get_logger(), "AckermannMuxNode created (unconfigured)");
}

AckermannMuxNode::CallbackReturn AckermannMuxNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  this->get_parameter_or("safety_threshold", safety_threshold_, 0.5);
  this->get_parameter_or("publish_rate_hz", publish_rate_hz_, 50.0);

  this->get_parameter_or("emergency.frame_id", emergency_.header.frame_id, std::string{""});
  this->get_parameter_or("emergency.steering_angle", emergency_.drive.steering_angle, 0.0f);
  this->get_parameter_or("emergency.speed", emergency_.drive.speed, 0.0f);

  this->get_parameter_or("fault_estop.enabled", fault_estop_enabled_, false);
  this->get_parameter_or("fault_estop.topic", fault_estop_topic_, std::string{"emergency_stop"});
  this->get_parameter_or("fault_estop.timeout_s", fault_estop_timeout_s_, 0.0);
  this->get_parameter_or("fault_estop.bypass_inputs", fault_estop_bypass_inputs_, std::vector<std::string>{"joystick"});

  pub_out_ =
    this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann", rclcpp::QoS(rclcpp::KeepLast(1)));

  if (fault_estop_enabled_) {
    // Durability must match fault_collector's latched emergency_stop publisher
    // (KeepLast(1) + transient_local) or this subscription will never connect.
    fault_estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      fault_estop_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      std::bind(&AckermannMuxNode::fault_estop_callback, this, std::placeholders::_1));
  }

  build_inputs_from_params();

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "publish_rate_hz must be > 0");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Configured: safety_threshold=%.3fs publish_rate=%.1fHz emergency(speed=%.3f, steer=%.3f) inputs=%zu "
    "fault_estop(enabled=%s, topic=%s, timeout=%.3fs, bypass_inputs=%zu)",
    safety_threshold_,
    publish_rate_hz_,
    emergency_.drive.speed,
    emergency_.drive.steering_angle,
    inputs_.size(),
    fault_estop_enabled_ ? "true" : "false",
    fault_estop_topic_.c_str(),
    fault_estop_timeout_s_,
    fault_estop_bypass_inputs_.size());

  return CallbackReturn::SUCCESS;
}

AckermannMuxNode::CallbackReturn AckermannMuxNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<nanoseconds>(period), std::bind(&AckermannMuxNode::ackerman_cmd_callback, this));

  RCLCPP_INFO(this->get_logger(), "Activated - mux running at %.1f Hz", publish_rate_hz_);
  return CallbackReturn::SUCCESS;
}

AckermannMuxNode::CallbackReturn AckermannMuxNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

AckermannMuxNode::CallbackReturn AckermannMuxNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  timer_.reset();
  pub_out_.reset();
  inputs_.clear();

  fault_estop_sub_.reset();
  {
    std::scoped_lock<std::mutex> lk(fault_estop_mtx_);
    fault_estop_value_ = false;
    has_fault_estop_msg_ = false;
    last_fault_estop_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  return CallbackReturn::SUCCESS;
}

AckermannMuxNode::CallbackReturn AckermannMuxNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  pub_out_.reset();
  inputs_.clear();

  fault_estop_sub_.reset();
  {
    std::scoped_lock<std::mutex> lk(fault_estop_mtx_);
    fault_estop_value_ = false;
    has_fault_estop_msg_ = false;
    last_fault_estop_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  return CallbackReturn::SUCCESS;
}

void AckermannMuxNode::build_inputs_from_params()
{
  const auto list = this->list_parameters({"inputs"}, 10);

  std::unordered_set<std::string> prefixes;
  for (const auto & pfx : list.prefixes) {
    if (pfx.rfind("inputs.", 0) == 0) {
      prefixes.insert(pfx);
    }
  }

  if (prefixes.empty()) {
    RCLCPP_WARN(this->get_logger(), "No inputs.* parameters found. Will always publish emergency.");
    return;
  }

  const std::unordered_set<std::string> bypass_set(
    fault_estop_bypass_inputs_.begin(), fault_estop_bypass_inputs_.end());

  for (const auto & prefix : prefixes) {
    const std::string name = prefix.substr(std::strlen("inputs."));

    InputConfig cfg;
    cfg.name = name;

    const auto topic_key = prefix + ".topic";
    if (!this->has_parameter(topic_key)) {
      RCLCPP_ERROR(this->get_logger(), "Input '%s' missing '%s'. Skipping.", cfg.name.c_str(), topic_key.c_str());
      continue;
    }
    cfg.topic = this->get_parameter(topic_key).as_string();
    if (cfg.topic.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Input '%s' has empty '%s'. Skipping.", cfg.name.c_str(), topic_key.c_str());
      continue;
    }

    const auto pri_key = prefix + ".priority";
    cfg.priority = this->has_parameter(pri_key) ? this->get_parameter(pri_key).as_int() : 0;

    const auto has_mask_key = prefix + ".has_mask";
    cfg.has_mask = this->has_parameter(has_mask_key) ? this->get_parameter(has_mask_key).as_bool() : false;

    const auto mask_topic_key = prefix + ".mask_topic";
    cfg.mask_topic = this->has_parameter(mask_topic_key) ? this->get_parameter(mask_topic_key).as_string() : "";

    const auto safety_gating_key = prefix + ".safety_gating";
    cfg.safety_gating =
      this->has_parameter(safety_gating_key) ? this->get_parameter(safety_gating_key).as_bool() : false;

    cfg.bypasses_fault_estop = bypass_set.count(cfg.name) > 0;

    inputs_.push_back(std::make_shared<InputHandle>(this, cfg));
  }
}

void AckermannMuxNode::fault_estop_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  std::scoped_lock<std::mutex> lk(fault_estop_mtx_);
  fault_estop_value_ = msg->data;
  has_fault_estop_msg_ = true;
  last_fault_estop_time_ = this->now();
}

bool AckermannMuxNode::is_fault_estop_asserted(const rclcpp::Time & now) const
{
  if (!fault_estop_enabled_) {
    return false;
  }

  std::scoped_lock<std::mutex> lk(fault_estop_mtx_);

  if (!has_fault_estop_msg_) {
    // No signal ever received from the fault collector. Only fail-safe if a staleness
    // timeout was configured; otherwise assume "not asserted" until the first message.
    return fault_estop_timeout_s_ > 0.0;
  }

  if (fault_estop_timeout_s_ > 0.0) {
    const double age_sec = (now - last_fault_estop_time_).seconds();
    if (age_sec > fault_estop_timeout_s_) {
      return true;  // Stale signal -> fail-safe assertion.
    }
  }

  return fault_estop_value_;
}

std::shared_ptr<InputHandle> AckermannMuxNode::select_fault_estop_bypass_input() const
{
  std::shared_ptr<InputHandle> winner;
  int64_t best_priority = std::numeric_limits<int64_t>::min();

  for (const auto & h : inputs_) {
    if (!h->cfg().bypasses_fault_estop) {
      continue;
    }
    if (!h->eligible_for_mux()) {
      continue;
    }
    const int64_t pri = h->cfg().priority;
    if (!winner || pri > best_priority) {
      winner = h;
      best_priority = pri;
    }
  }

  return winner;
}

void AckermannMuxNode::ackerman_cmd_callback()
{
  const auto now = this->now();

  // Fault-driven emergency stop is a hard override on top of priority arbitration and
  // safety gating below -- EXCEPT for inputs explicitly marked as bypassing it (typically
  // the joystick), so the safety driver is never locked out of the vehicle.
  if (is_fault_estop_asserted(now)) {
    auto bypass_winner = select_fault_estop_bypass_input();

    if (bypass_winner) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Fault emergency stop asserted; publishing bypass input '%s' so the operator retains control.",
        bypass_winner->cfg().name.c_str());
      auto out = bypass_winner->last_cmd();
      out.header.stamp = now;
      pub_out_->publish(out);
      return;
    }

    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Fault emergency stop asserted and no eligible bypass input available; publishing emergency command.");
    auto e = emergency_;
    e.header.stamp = now;
    pub_out_->publish(e);
    return;
  }

  // Safety override
  for (const auto & h : inputs_) {
    if (h->safety_trip(now, safety_threshold_)) {
      auto e = emergency_;
      e.header.stamp = now;
      pub_out_->publish(e);
      return;
    }
  }

  // Pick the highest-priority eligible input.
  std::shared_ptr<InputHandle> winner;
  int64_t best_priority = std::numeric_limits<int64_t>::min();

  for (const auto & h : inputs_) {
    if (!h->eligible_for_mux()) {
      continue;
    }
    const int64_t pri = h->cfg().priority;
    if (!winner || pri > best_priority) {
      winner = h;
      best_priority = pri;
    }
  }

  // No eligible inputs: publish emergency.
  if (!winner) {
    auto e = emergency_;
    e.header.stamp = now;
    pub_out_->publish(e);
    return;
  }

  // Publish the latest command from the winning input, stamped as "now".
  auto out = winner->last_cmd();
  out.header.stamp = now;
  pub_out_->publish(out);
}

}  // namespace ackermann_mux

RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_mux::AckermannMuxNode)
