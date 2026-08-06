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

#include "fault_collector/fault_collector_node.hpp"

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using std::chrono::nanoseconds;

namespace fault_collector
{

FaultCollectorNode::FaultCollectorNode(const rclcpp::NodeOptions & options)
: LifecycleNode("fault_collector", options)
{
  declare_parameter("faults_topic", std::string("faults"));
  declare_parameter("diagnostics_topic", std::string("/diagnostics"));
  declare_parameter("emergency_stop_topic", std::string("emergency_stop"));
  declare_parameter("autonomy_disarm_topic", std::string("autonomy_disarm"));
  declare_parameter("arm_service", std::string("/oscc_interfacing/arm"));
  declare_parameter("is_armed_topic", std::string("/oscc_interfacing/is_armed"));
  declare_parameter("min_disarm_interval_s", 0.5);
  declare_parameter("publish_rate_hz", 5.0);
  declare_parameter("fault_timeout_s", 5.0);
  declare_parameter("disarm_level", 1);
  declare_parameter("brake_level", 2);
  declare_parameter("latch", true);
  declare_parameter("disarm_enabled", true);
  declare_parameter("diagnostic_name_prefix", std::string("fault_collector"));
  declare_parameter("diagnostic_level_map", std::vector<int64_t>{2, 2, 1, 0});

  RCLCPP_INFO(this->get_logger(), "FaultCollectorNode created (unconfigured)");
}

FaultCollectorNode::CallbackReturn FaultCollectorNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  faults_topic_ = get_parameter("faults_topic").as_string();
  diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();
  emergency_stop_topic_ = get_parameter("emergency_stop_topic").as_string();
  autonomy_disarm_topic_ = get_parameter("autonomy_disarm_topic").as_string();
  arm_service_ = get_parameter("arm_service").as_string();
  is_armed_topic_ = get_parameter("is_armed_topic").as_string();
  min_disarm_interval_s_ = get_parameter("min_disarm_interval_s").as_double();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  fault_timeout_s_ = get_parameter("fault_timeout_s").as_double();
  disarm_level_ = get_parameter("disarm_level").as_int();
  brake_level_ = get_parameter("brake_level").as_int();
  latch_ = get_parameter("latch").as_bool();
  disarm_enabled_ = get_parameter("disarm_enabled").as_bool();
  diagnostic_name_prefix_ = get_parameter("diagnostic_name_prefix").as_string();
  const std::vector<int64_t> diagnostic_level_map = get_parameter("diagnostic_level_map").as_integer_array();

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "publish_rate_hz must be > 0");
    return CallbackReturn::FAILURE;
  }

  if (fault_timeout_s_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "fault_timeout_s must be > 0");
    return CallbackReturn::FAILURE;
  }

  if (min_disarm_interval_s_ < 0.0) {
    RCLCPP_ERROR(this->get_logger(), "min_disarm_interval_s must be >= 0 (got %.3f)", min_disarm_interval_s_);
    return CallbackReturn::FAILURE;
  }

  if (disarm_level_ < kCritical || disarm_level_ > kMild) {
    RCLCPP_ERROR(
      this->get_logger(), "disarm_level must be in 1..4 (got %" PRId64 ")", static_cast<int64_t>(disarm_level_));
    return CallbackReturn::FAILURE;
  }

  if (brake_level_ < kCritical || brake_level_ > kMild) {
    RCLCPP_ERROR(
      this->get_logger(), "brake_level must be in 1..4 (got %" PRId64 ")", static_cast<int64_t>(brake_level_));
    return CallbackReturn::FAILURE;
  }

  if (disarm_level_ > brake_level_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "disarm_level (%" PRId64 ") must be <= brake_level (%" PRId64
      "): disarm is Tier A and must be at least as "
      "strict as "
      "Tier B brake",
      static_cast<int64_t>(disarm_level_),
      static_cast<int64_t>(brake_level_));
    return CallbackReturn::FAILURE;
  }

  if (diagnostic_level_map.size() != 4) {
    RCLCPP_ERROR(
      this->get_logger(), "diagnostic_level_map must have exactly 4 entries (got %zu)", diagnostic_level_map.size());
    return CallbackReturn::FAILURE;
  }

  FaultCollectorConfig config;
  config.fault_timeout_s = fault_timeout_s_;
  config.disarm_level = static_cast<uint8_t>(disarm_level_);
  config.brake_level = static_cast<uint8_t>(brake_level_);
  config.latch = latch_;

  for (size_t i = 0; i < 4; ++i) {
    const int64_t v = diagnostic_level_map[i];
    if (v < 0 || v > 3) {
      RCLCPP_ERROR(
        this->get_logger(),
        "diagnostic_level_map[%zu] must be in 0..3 (OK=0, WARN=1, ERROR=2, STALE=3), got %" PRId64,
        i,
        static_cast<int64_t>(v));
      return CallbackReturn::FAILURE;
    }
    config.diagnostic_level_map[i] = static_cast<uint8_t>(v);
  }

  // Belt-and-suspenders: config.is_valid() re-checks the same invariant as above.
  if (!config.is_valid()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid fault collector configuration: disarm_level must be <= brake_level");
    return CallbackReturn::FAILURE;
  }

  diagnostic_level_map_ = config.diagnostic_level_map;
  core_ = std::make_unique<FaultCollectorCore>(config);
  last_brake_state_ = false;
  disarm_failed_ = false;

  fault_sub_ = this->create_subscription<wato_fault_msgs::msg::Fault>(
    faults_topic_,
    rclcpp::QoS(rclcpp::KeepLast(100)),
    std::bind(&FaultCollectorNode::fault_callback, this, std::placeholders::_1));

  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_, rclcpp::QoS(rclcpp::KeepLast(10)));

  emergency_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    emergency_stop_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  autonomy_disarm_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    autonomy_disarm_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  arm_client_ = this->create_client<std_srvs::srv::SetBool>(arm_service_);

  // Watch oscc_interfacing's arm state so a re-arm during a live disarm-level fault can be
  // undone immediately. This is the re-engage lockout; oscc_interfacing itself is unmodified.
  is_armed_ = false;
  has_is_armed_msg_ = false;
  last_disarm_call_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  is_armed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    is_armed_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&FaultCollectorNode::is_armed_callback, this, std::placeholders::_1));

  clear_faults_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/clear_faults",
    std::bind(&FaultCollectorNode::handle_clear_faults, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    this->get_logger(),
    "Configured: faults_topic=%s diagnostics_topic=%s emergency_stop_topic=%s autonomy_disarm_topic=%s "
    "arm_service=%s is_armed_topic=%s publish_rate=%.1fHz fault_timeout=%.3fs disarm_level=%" PRId64
    " (%s) "
    "brake_level=%" PRId64
    " "
    "(%s) "
    "latch=%s disarm_enabled=%s min_disarm_interval_s=%.3f",
    faults_topic_.c_str(),
    diagnostics_topic_.c_str(),
    emergency_stop_topic_.c_str(),
    autonomy_disarm_topic_.c_str(),
    arm_service_.c_str(),
    is_armed_topic_.c_str(),
    publish_rate_hz_,
    fault_timeout_s_,
    static_cast<int64_t>(disarm_level_),
    severity_name(static_cast<uint8_t>(disarm_level_)).c_str(),
    static_cast<int64_t>(brake_level_),
    severity_name(static_cast<uint8_t>(brake_level_)).c_str(),
    latch_ ? "true" : "false",
    disarm_enabled_ ? "true" : "false",
    min_disarm_interval_s_);

  return CallbackReturn::SUCCESS;
}

FaultCollectorNode::CallbackReturn FaultCollectorNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<nanoseconds>(period), std::bind(&FaultCollectorNode::publish_diagnostics, this));

  RCLCPP_INFO(this->get_logger(), "Activated - fault collector running at %.1f Hz", publish_rate_hz_);
  return CallbackReturn::SUCCESS;
}

FaultCollectorNode::CallbackReturn FaultCollectorNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

FaultCollectorNode::CallbackReturn FaultCollectorNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  timer_.reset();
  fault_sub_.reset();
  is_armed_sub_.reset();
  diagnostics_pub_.reset();
  emergency_stop_pub_.reset();
  autonomy_disarm_pub_.reset();
  arm_client_.reset();
  is_armed_ = false;
  has_is_armed_msg_ = false;
  last_disarm_call_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  clear_faults_srv_.reset();
  core_.reset();
  last_brake_state_ = false;
  disarm_failed_ = false;

  return CallbackReturn::SUCCESS;
}

FaultCollectorNode::CallbackReturn FaultCollectorNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  fault_sub_.reset();
  is_armed_sub_.reset();
  diagnostics_pub_.reset();
  emergency_stop_pub_.reset();
  autonomy_disarm_pub_.reset();
  arm_client_.reset();
  is_armed_ = false;
  has_is_armed_msg_ = false;
  last_disarm_call_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  clear_faults_srv_.reset();
  core_.reset();
  last_brake_state_ = false;
  disarm_failed_ = false;

  return CallbackReturn::SUCCESS;
}

void FaultCollectorNode::fault_callback(const wato_fault_msgs::msg::Fault::ConstSharedPtr msg)
{
  FaultRecord record;
  record.level = msg->level;
  record.source = msg->source;
  record.message = msg->message;
  record.id = msg->id;
  record.active = msg->active;

  const double now_s = this->now().seconds();
  const ReportResult result = core_->report(record, now_s);

  switch (result) {
    case ReportResult::kRejectedInvalidLevel:
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected fault report from source='%s': invalid level %u (expected 1..4)",
        msg->source.c_str(),
        static_cast<unsigned>(msg->level));
      break;
    case ReportResult::kRejectedEmptySource:
      RCLCPP_WARN(this->get_logger(), "Rejected fault report with empty source (message='%s')", msg->message.c_str());
      break;
    case ReportResult::kAccepted:
    case ReportResult::kCleared:
      break;
  }
}

std::string FaultCollectorNode::describe_faults_at_or_below(uint8_t level) const
{
  std::string offenders;
  for (const auto & [key, fault] : core_->active_faults()) {
    (void)key;
    if (fault.level > level) {
      continue;
    }
    if (!offenders.empty()) {
      offenders += ", ";
    }
    const std::string id_or_message = fault.id.empty() ? fault.message : fault.id;
    offenders += fault.source + ": " + id_or_message;
  }
  return offenders.empty() ? "(latched; offending fault no longer active)" : offenders;
}

void FaultCollectorNode::is_armed_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  const bool was_armed = has_is_armed_msg_ && is_armed_;
  is_armed_ = msg->data;
  has_is_armed_msg_ = true;

  if (!disarm_enabled_) {
    return;
  }

  // Rising edge into armed while a disarm-level fault is still active or latched: the operator
  // has tried to re-engage out of a live fault. Put it straight back down.
  if (!was_armed && is_armed_ && core_ && core_->disarm_required()) {
    request_disarm("re-arm attempted while a disarm-level fault is active or latched");
  }
}

void FaultCollectorNode::request_disarm(const char * reason)
{
  const auto now = this->now();

  // Bound the service traffic so a repeated arm/disarm fight cannot flood oscc_interfacing.
  if (last_disarm_call_time_.nanoseconds() != 0) {
    const double since_last_s = (now - last_disarm_call_time_).seconds();
    if (since_last_s < min_disarm_interval_s_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Suppressing disarm request (%.3fs since last, minimum %.3fs): %s",
        since_last_s,
        min_disarm_interval_s_,
        reason);
      return;
    }
  }

  last_disarm_call_time_ = now;

  RCLCPP_ERROR(this->get_logger(), "Requesting vehicle disarm via '%s': %s", arm_service_.c_str(), reason);

  if (!arm_client_ || !arm_client_->service_is_ready()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Arm service '%s' is not ready; cannot disarm. Forcing emergency_stop as a fallback.",
      arm_service_.c_str());
    disarm_failed_ = true;
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  arm_client_->async_send_request(
    request, std::bind(&FaultCollectorNode::handle_disarm_response, this, std::placeholders::_1));
}

void FaultCollectorNode::handle_disarm_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
{
  const auto response = future.get();

  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Disarm request to '%s' was rejected: %s. Forcing emergency_stop as a fallback.",
      arm_service_.c_str(),
      response->message.c_str());
    disarm_failed_ = true;
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Vehicle disarm requested via '%s' succeeded: %s",
      arm_service_.c_str(),
      response->message.c_str());
    disarm_failed_ = false;
  }
}

void FaultCollectorNode::publish_diagnostics()
{
  const double now_s = this->now().seconds();
  core_->expire(now_s);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();

  size_t critical_count = 0;
  size_t severe_count = 0;
  size_t moderate_count = 0;
  size_t mild_count = 0;

  for (const auto & [key, fault] : core_->active_faults()) {
    (void)key;

    switch (fault.level) {
      case kCritical:
        ++critical_count;
        break;
      case kSevere:
        ++severe_count;
        break;
      case kModerate:
        ++moderate_count;
        break;
      case kMild:
        ++mild_count;
        break;
      default:
        break;
    }

    const std::string id_or_message = fault.id.empty() ? fault.message : fault.id;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = diagnostic_name_prefix_ + ": " + fault.source + ": " + id_or_message;
    status.hardware_id = fault.source;
    status.level = diagnostic_level_map_[fault.level - 1];
    status.message = fault.message;

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "level";
    kv.value = std::to_string(fault.level) + " (" + severity_name(fault.level) + ")";
    status.values.push_back(kv);

    kv.key = "source";
    kv.value = fault.source;
    status.values.push_back(kv);

    kv.key = "id";
    kv.value = fault.id;
    status.values.push_back(kv);

    kv.key = "count";
    kv.value = std::to_string(fault.count);
    status.values.push_back(kv);

    kv.key = "age_s";
    kv.value = std::to_string(now_s - fault.last_seen_s);
    status.values.push_back(kv);

    kv.key = "first_seen_s";
    kv.value = std::to_string(fault.first_seen_s);
    status.values.push_back(kv);

    array.status.push_back(status);
  }

  const auto worst = core_->worst_level();

  // Tier A: fire the disarm request on the rising edge. Sustained enforcement (stopping the
  // operator arming back out of a live fault) is handled by is_armed_callback().
  if (disarm_enabled_ && core_->consume_disarm_edge()) {
    request_disarm(describe_faults_at_or_below(static_cast<uint8_t>(disarm_level_)).c_str());
  }

  const bool disarm_requested = core_->should_disarm();
  const bool brake_requested = core_->should_brake();
  const bool emergency_stop = brake_requested || disarm_failed_;
  const bool disarm_service_ok = arm_client_ && arm_client_->service_is_ready() && !disarm_failed_;

  diagnostic_msgs::msg::DiagnosticStatus summary;
  summary.name = diagnostic_name_prefix_ + ": summary";
  summary.hardware_id = this->get_name();

  if (worst.has_value()) {
    std::string worst_source;
    for (const auto & [key, fault] : core_->active_faults()) {
      (void)key;
      if (fault.level == *worst) {
        worst_source = fault.source;
        break;
      }
    }
    summary.level = diagnostic_level_map_[*worst - 1];
    summary.message = std::to_string(core_->active_faults().size()) +
                      " active fault(s), worst=" + severity_name(*worst) + " (" + worst_source + ")";
  } else {
    summary.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    summary.message = "No active faults";
  }

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "active_faults";
  kv.value = std::to_string(core_->active_faults().size());
  summary.values.push_back(kv);

  kv.key = "worst_level";
  kv.value = worst.has_value() ? (std::to_string(*worst) + " (" + severity_name(*worst) + ")") : "none";
  summary.values.push_back(kv);

  kv.key = "disarm_requested";
  kv.value = disarm_requested ? "true" : "false";
  summary.values.push_back(kv);

  kv.key = "emergency_stop";
  kv.value = emergency_stop ? "true" : "false";
  summary.values.push_back(kv);

  kv.key = "is_armed";
  kv.value = has_is_armed_msg_ ? (is_armed_ ? "true" : "false") : "unknown";
  summary.values.push_back(kv);

  kv.key = "disarm_service_ok";
  kv.value = disarm_service_ok ? "true" : "false";
  summary.values.push_back(kv);

  kv.key = "critical";
  kv.value = std::to_string(critical_count);
  summary.values.push_back(kv);

  kv.key = "severe";
  kv.value = std::to_string(severe_count);
  summary.values.push_back(kv);

  kv.key = "moderate";
  kv.value = std::to_string(moderate_count);
  summary.values.push_back(kv);

  kv.key = "mild";
  kv.value = std::to_string(mild_count);
  summary.values.push_back(kv);

  array.status.insert(array.status.begin(), summary);

  diagnostics_pub_->publish(array);

  std_msgs::msg::Bool disarm_status_msg;
  disarm_status_msg.data = disarm_requested;
  autonomy_disarm_pub_->publish(disarm_status_msg);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = emergency_stop;
  emergency_stop_pub_->publish(estop_msg);

  if (emergency_stop) {
    if (disarm_failed_) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "emergency_stop forced true: disarm attempt failed and the vehicle could not be confirmed disarmed.");
    } else {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "emergency_stop asserted due to fault(s): %s",
        describe_faults_at_or_below(static_cast<uint8_t>(brake_level_)).c_str());
    }
  } else if (last_brake_state_) {
    RCLCPP_INFO(this->get_logger(), "emergency_stop cleared; resuming normal operation.");
  }
  last_brake_state_ = emergency_stop;
}

void FaultCollectorNode::handle_clear_faults(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const size_t count = core_->active_faults().size();
  core_->clear_all();
  core_->clear_latch();
  disarm_failed_ = false;

  response->success = true;
  response->message =
    "Cleared " + std::to_string(count) + " fault(s), released both latches, and reset the disarm-failure flag.";

  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

}  // namespace fault_collector

RCLCPP_COMPONENTS_REGISTER_NODE(fault_collector::FaultCollectorNode)
