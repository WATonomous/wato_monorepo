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

#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>

#include <rclcpp_components/register_node_macro.hpp>

using std::chrono::nanoseconds;

namespace ackermann_mux
{

AckermannMuxNode::AckermannMuxNode(const rclcpp::NodeOptions & options)
: Node(
    "ackermann_mux",
    rclcpp::NodeOptions(options).allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
      true))
{
  safety_threshold_ = get_param_or<double>("safety_threshold", 0.5);
  publish_rate_hz_ = get_param_or<double>("publish_rate_hz", 50.0);

  emergency_.drive.steering_angle = get_param_or<double>("emergency.steering_angle", 0.0);
  emergency_.drive.speed = get_param_or<double>("emergency.speed", 0.0);

  emergency_.header.frame_id = "";
  emergency_.drive.steering_angle = get_param_or<double>("emergency.steering_angle", 0.0);
  emergency_.drive.speed = get_param_or<double>("emergency.speed", 0.0);
  pub_out_ =
    this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann", rclcpp::QoS(rclcpp::KeepLast(1)));

  build_inputs_from_params();

  if (publish_rate_hz_ <= 0.0) {
    publish_rate_hz_ = 50.0;
  }
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<nanoseconds>(period), std::bind(&AckermannMuxNode::ackerman_cmd_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "ackermann_mux component running: safety_threshold=%.3fs publish_rate=%.1fHz emergency(speed=%.3f, steer=%.3f) "
    "inputs=%zu",
    safety_threshold_,
    publish_rate_hz_,
    emergency_.drive.speed,
    emergency_.drive.steering_angle,
    inputs_.size());
}

template <typename T>
T AckermannMuxNode::get_param_or(const std::string & name, const T & fallback) const
{
  if (!this->has_parameter(name)) {
    return fallback;
  }
  const auto p = this->get_parameter(name);

  if constexpr (std::is_same_v<T, std::string>) {
    return p.as_string();
  } else if constexpr (std::is_same_v<T, bool>) {
    return p.as_bool();
  } else if constexpr (std::is_same_v<T, int>) {
    return static_cast<int>(p.as_int());
  } else if constexpr (std::is_same_v<T, int64_t>) {
    return p.as_int();
  } else if constexpr (std::is_same_v<T, double>) {
    return p.as_double();
  } else {
    return fallback;
  }
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

  for (const auto & prefix : prefixes) {
    const std::string name = prefix.substr(std::strlen("inputs."));

    InputConfig cfg;
    cfg.name = name;
    cfg.topic = get_param_or<std::string>(prefix + ".topic", "");
    cfg.priority = get_param_or<int>(prefix + ".priority", 0);
    cfg.has_mask = get_param_or<bool>(prefix + ".has_mask", false);
    cfg.mask_topic = get_param_or<std::string>(prefix + ".mask_topic", "");
    cfg.safety_gating = get_param_or<bool>(prefix + ".safety_gating", false);

    if (cfg.topic.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Input '%s' missing '%s.topic'. Skipping.", cfg.name.c_str(), prefix.c_str());
      continue;
    }

    inputs_.push_back(std::make_shared<InputHandle>(this, cfg));
  }
}

void AckermannMuxNode::ackerman_cmd_callback()
{
  const auto now = this->now();

  // 1) Safety override
  for (const auto & h : inputs_) {
    if (h->safety_trip(now, safety_threshold_)) {
      auto e = emergency_;
      e.header.stamp = now;
      pub_out_->publish(e);
      return;
    }
  }

  // 2) Arbitration
  std::shared_ptr<InputHandle> winner = nullptr;
  int best_priority = std::numeric_limits<int>::min();

  for (const auto & h : inputs_) {
    if (!h->eligible_for_mux()) {
      continue;
    }
    const int pri = h->cfg().priority;
    if (!winner || pri > best_priority) {
      winner = h;
      best_priority = pri;
    }
  }

  // 3) No eligible => emergency
  if (!winner) {
    auto e = emergency_;
    e.header.stamp = now;
    pub_out_->publish(e);
    return;
  }

  // 4) Publish winner latest cmd (update stamp to now so downstream sees "fresh")
  auto out = winner->last_cmd();
  out.header.stamp = now;
  pub_out_->publish(out);
}

}  // namespace ackermann_mux

RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_mux::AckermannMuxNode)
