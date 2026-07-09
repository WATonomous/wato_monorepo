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

#include "oscc_mux/oscc_mux_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>

#include <rclcpp_components/register_node_macro.hpp>

using std::chrono::nanoseconds;

namespace oscc_mux
{

OsccMuxNode::OsccMuxNode(const rclcpp::NodeOptions & options)
: LifecycleNode(
    "oscc_mux",
    rclcpp::NodeOptions(options).allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
      true))
{
  RCLCPP_INFO(this->get_logger(), "OsccMuxNode created (unconfigured)");
}

OsccMuxNode::CallbackReturn OsccMuxNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  this->get_parameter_or("safety_threshold", safety_threshold_, 0.5);
  this->get_parameter_or("publish_rate_hz", publish_rate_hz_, 50.0);

  this->get_parameter_or("emergency.steering", emergency_.steering, 0.0f);
  this->get_parameter_or("emergency.forward", emergency_.forward, 0.0f);

  this->get_parameter_or("enable_handover_ramp", enable_handover_ramp_, true);
  this->get_parameter_or("handover_ramp_ms", handover_ramp_ms_, 600.0);
  if (handover_ramp_ms_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "handover_ramp_ms must be > 0, disabling handover ramp");
    enable_handover_ramp_ = false;
  }
  last_published_ = emergency_;
  previous_winner_.reset();
  in_handover_ = false;

  pub_out_ = this->create_publisher<roscco_msg::msg::Roscco>("roscco", rclcpp::QoS(rclcpp::KeepLast(1)));

  build_inputs_from_params();

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "publish_rate_hz must be > 0");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Configured: safety_threshold=%.3fs publish_rate=%.1fHz emergency(forward=%.3f, steering=%.3f) "
    "handover_ramp=%s (%.0fms) inputs=%zu",
    safety_threshold_,
    publish_rate_hz_,
    emergency_.forward,
    emergency_.steering,
    enable_handover_ramp_ ? "enabled" : "disabled",
    handover_ramp_ms_,
    inputs_.size());

  return CallbackReturn::SUCCESS;
}

OsccMuxNode::CallbackReturn OsccMuxNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<nanoseconds>(period), std::bind(&OsccMuxNode::mux_callback, this));

  RCLCPP_INFO(this->get_logger(), "Activated - mux running at %.1f Hz", publish_rate_hz_);
  return CallbackReturn::SUCCESS;
}

OsccMuxNode::CallbackReturn OsccMuxNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

OsccMuxNode::CallbackReturn OsccMuxNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  timer_.reset();
  pub_out_.reset();
  inputs_.clear();

  previous_winner_.reset();
  in_handover_ = false;
  last_published_ = roscco_msg::msg::Roscco{};

  return CallbackReturn::SUCCESS;
}

OsccMuxNode::CallbackReturn OsccMuxNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  pub_out_.reset();
  inputs_.clear();

  previous_winner_.reset();
  in_handover_ = false;
  last_published_ = roscco_msg::msg::Roscco{};

  return CallbackReturn::SUCCESS;
}

void OsccMuxNode::build_inputs_from_params()
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

    inputs_.push_back(std::make_shared<InputHandle>(this, cfg));
  }
}

void OsccMuxNode::mux_callback()
{
  const auto now = this->now();

  // Safety override — bypass the handover ramp, must take effect instantly.
  // Reset previous_winner_ so a later return to normal operation starts a
  // fresh handover from this emergency baseline rather than a stale value.
  for (const auto & h : inputs_) {
    if (h->safety_trip(now, safety_threshold_)) {
      auto e = emergency_;
      e.header.stamp = now;
      pub_out_->publish(e);
      last_published_ = e;
      previous_winner_.reset();
      in_handover_ = false;
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

  // No eligible inputs: publish emergency, bypassing the ramp for the same reason as above.
  if (!winner) {
    auto e = emergency_;
    e.header.stamp = now;
    pub_out_->publish(e);
    last_published_ = e;
    previous_winner_.reset();
    in_handover_ = false;
    return;
  }

  // Winner changed since the last tick: start a handover ramp from the last
  // published command toward this winner's live command.
  if (enable_handover_ramp_ && winner != previous_winner_ && previous_winner_ != nullptr) {
    begin_handover();
  }
  previous_winner_ = winner;

  const auto target = winner->last_cmd();
  roscco_msg::msg::Roscco out;
  if (in_handover_) {
    const float a = handover_authority_scale();
    out.forward = handover_start_cmd_.forward + (target.forward - handover_start_cmd_.forward) * a;
    out.steering = handover_start_cmd_.steering + (target.steering - handover_start_cmd_.steering) * a;
    tick_handover();
  } else {
    out.forward = target.forward;
    out.steering = target.steering;
  }
  out.header.stamp = now;
  pub_out_->publish(out);
  last_published_ = out;
}

void OsccMuxNode::begin_handover()
{
  handover_start_cmd_ = last_published_;
  handover_start_time_ = this->now();
  in_handover_ = true;
}

float OsccMuxNode::handover_authority_scale() const
{
  if (!in_handover_) {
    return 1.0f;
  }
  const double elapsed_ms = (this->now() - handover_start_time_).seconds() * 1000.0;
  const double frac = elapsed_ms / handover_ramp_ms_;
  return static_cast<float>(std::clamp(frac, 0.0, 1.0));
}

void OsccMuxNode::tick_handover()
{
  if (!in_handover_) {
    return;
  }
  const double elapsed_ms = (this->now() - handover_start_time_).seconds() * 1000.0;
  if (elapsed_ms >= handover_ramp_ms_) {
    in_handover_ = false;
  }
}

}  // namespace oscc_mux

RCLCPP_COMPONENTS_REGISTER_NODE(oscc_mux::OsccMuxNode)
