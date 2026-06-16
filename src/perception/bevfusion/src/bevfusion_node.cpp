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

#include "bevfusion/bevfusion_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/image_marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace wato::perception::bevfusion
{

BEVFusionNode::BEVFusionNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("bevfusion_node", options)
, subscriber_qos_(10)
, publisher_qos_(10)
, last_stats_log_time_(std::chrono::steady_clock::now())
{
  RCLCPP_INFO(this->get_logger(), "BEVFusion ROS 2 lifecycle node created");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", this->get_current_state().label().c_str());
}

void BEVFusionNode::declareParameters()
{
  core_ = std::make_unique<BEVFusionCore>();
}

void BEVFusionNode::updateStatistics(double time_taken)
{
  total_processed_++;
  double current_total = total_processing_time_ms_.load();
  while (!total_processing_time_ms_.compare_exchange_weak(current_total, current_total + time_taken)) {
  }
  last_processing_time_ms_ = time_taken;

  const auto now = std::chrono::steady_clock::now();
  if (now - last_stats_log_time_ >= kStatsLogInterval) {
    logStatistics();
    last_stats_log_time_ = now;
  }
}

void BEVFusionNode::updateDiagnostics(const std_msgs::msg::Header::_stamp_type & timestamp)
{
  if (pub_diagnostic_) {
    pub_diagnostic_->tick(timestamp);
  }

  if (diagnostic_updater_) {
    diagnostic_updater_->force_update();
  }
}

void BEVFusionNode::logStatistics() const
{
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;

  RCLCPP_INFO(
    this->get_logger(), "Statistics: %lu arrays processed, average processing time: %.3f ms", processed, avg_time);
}

rclcpp::QoS BEVFusionNode::createSubscriberQoS(const std::string & reliability, int depth)
{
  RCLCPP_INFO(this->get_logger(), "Creating subscriber QoS: reliability='%s', depth=%d", reliability.c_str(), depth);

  rclcpp::QoS qos(depth);
  if (reliability == "reliable") {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  } else if (reliability == "best_effort") {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown reliability policy '%s', defaulting to best_effort. Valid values: 'reliable', 'best_effort'",
      reliability.c_str());
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  }
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  return qos;
}

rclcpp::QoS BEVFusionNode::createPublisherQoS(
  const std::string & reliability, const std::string & durability, int depth)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Creating publisher QoS: reliability='%s', durability='%s', depth=%d",
    reliability.c_str(),
    durability.c_str(),
    depth);

  rclcpp::QoS qos(depth);
  if (reliability == "reliable") {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  } else if (reliability == "best_effort") {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown reliability policy '%s', defaulting to reliable. Valid values: 'reliable', 'best_effort'",
      reliability.c_str());
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  }

  if (durability == "transient_local") {
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  } else if (durability == "volatile") {
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown durability policy '%s', defaulting to transient_local. Valid values: 'transient_local', 'volatile'",
      durability.c_str());
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
  return qos;
}

void BEVFusionNode::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;
  const double last_time = last_processing_time_ms_.load();

  if (processed == 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No ___TODO(ashish)____ processed yet");
  } else if (avg_time > 50.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "High processing latency");
  } else if (last_time > 100.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Recent high processing latency");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Operating normally");
  }

  stat.add("Arrays Processed", processed);
  stat.add("Average Processing Time (ms)", avg_time);
  stat.add("Last Processing Time (ms)", last_time);
}

// ---------------------------------------------------------------------------
// Lifecycle callbacks
// ---------------------------------------------------------------------------

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BEVFusionNode::on_configure(
  const rclcpp_lifecycle::State & /* prev_state */)
{
  RCLCPP_INFO(this->get_logger(), "Configuring BEVFusion node");

  try {
    declareParameters();

    // Log configuration
    RCLCPP_INFO(this->get_logger(), "Configuration summary: (EMPTY RIGHT NOW)");

    // TODO(ashish): Declare topic name parameters (for message_filters - remapping doesn't work automatically)

    // Declare and configure QoS parameters
    this->declare_parameter<std::string>("qos_subscriber_reliability", "best_effort");
    this->declare_parameter<int>("qos_subscriber_depth", 10);
    this->declare_parameter<std::string>("qos_publisher_reliability", "reliable");
    this->declare_parameter<std::string>("qos_publisher_durability", "transient_local");
    this->declare_parameter<int>("qos_publisher_depth", 10);

    const std::string subscriber_reliability = this->get_parameter("qos_subscriber_reliability").as_string();
    const int subscriber_depth = this->get_parameter("qos_subscriber_depth").as_int();
    subscriber_qos_ = createSubscriberQoS(subscriber_reliability, subscriber_depth);

    const std::string publisher_reliability = this->get_parameter("qos_publisher_reliability").as_string();
    const std::string publisher_durability = this->get_parameter("qos_publisher_durability").as_string();
    const int publisher_depth = this->get_parameter("qos_publisher_depth").as_int();
    publisher_qos_ = createPublisherQoS(publisher_reliability, publisher_durability, publisher_depth);

    this->declare_parameter<int>("sync_queue_size", 10);
    this->declare_parameter<double>("sync_max_time_diff_ms", 200.0);

    sync_queue_size_ = this->get_parameter("sync_queue_size").as_int();
    sync_max_time_diff_ms_ = this->get_parameter("sync_max_time_diff_ms").as_double();
    sync_max_time_diff_sec_ = sync_max_time_diff_ms_ / 1000.0;  // Convert to seconds

    RCLCPP_INFO(this->get_logger(), "Message synchronization settings:");
    RCLCPP_INFO(this->get_logger(), "  - Queue size: %d", sync_queue_size_);
    RCLCPP_INFO(
      this->get_logger(),
      "  - Max time difference: %.1f ms (%.3f sec)",
      sync_max_time_diff_ms_,
      sync_max_time_diff_sec_);
    RCLCPP_INFO(this->get_logger(), "  - Synchronization method: ApproximateTime");

    // Initialize diagnostics
    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("bevfusion");
    diagnostic_updater_->add("BEVFusion Status", this, &BEVFusionNode::diagnosticCallback);

    RCLCPP_INFO(this->get_logger(), "Node configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Configuration failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BEVFusionNode::on_activate(
  const rclcpp_lifecycle::State & prev_state)
{
  RCLCPP_INFO(this->get_logger(), "Activating BEVFusion node");
  RCLCPP_INFO(this->get_logger(), "Previous state: %s", prev_state.label().c_str());
  RCLCPP_INFO(this->get_logger(), "=============================================");

  try {
    // TODO(ashish): Create message_filters subscribers for ApproximateTime sync
    RCLCPP_INFO(this->get_logger(), "Creating subscribers:");

    RCLCPP_INFO(this->get_logger(), "=============================================");
    RCLCPP_INFO(this->get_logger(), "Node activated successfully!");
    RCLCPP_INFO(this->get_logger(), "Subscribed to:");
    // TODO(ashish)
    RCLCPP_INFO(this->get_logger(), "Publishing to:");
    // TODO(ashish)
    RCLCPP_INFO(
      this->get_logger(),
      "[SYNC] ApproximateTime sync (queue=%d, max=%.3fs)",
      sync_queue_size_,
      sync_max_time_diff_sec_);
    RCLCPP_INFO(this->get_logger(), "=============================================");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Activation failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BEVFusionNode::on_deactivate(
  const rclcpp_lifecycle::State & /* prev_state */)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating BEVFusion node");

  // TOOD

  RCLCPP_INFO(this->get_logger(), "=============================================");
  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BEVFusionNode::on_cleanup(
  const rclcpp_lifecycle::State & /* prev_state */)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up BEVFusion node");

  // TODO(ashish)

  RCLCPP_INFO(this->get_logger(), "Cleaning up publishers...");
  // TODO(ashish)
  core_.reset();

  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BEVFusionNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "=============================================");
  RCLCPP_INFO(this->get_logger(), "Shutting down BEVFusion node");
  RCLCPP_INFO(this->get_logger(), "Previous state: %s", previous_state.label().c_str());
  RCLCPP_INFO(this->get_logger(), "=============================================");

  // Log final statistics before shutdown
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;

  RCLCPP_INFO(this->get_logger(), "Final statistics:");
  RCLCPP_INFO(this->get_logger(), "  - Total arrays processed: %lu", processed);
  RCLCPP_INFO(this->get_logger(), "  - Total processing time: %.3f ms", total_time);
  RCLCPP_INFO(this->get_logger(), "  - Average processing time: %.3f ms", avg_time);
  RCLCPP_INFO(this->get_logger(), "Message statistics:");
  // TODO(ashish)

  RCLCPP_INFO(this->get_logger(), "Resetting all resources...");

  core_.reset();
  pub_diagnostic_.reset();
  diagnostic_updater_.reset();

  total_processed_ = 0;
  total_processing_time_ms_ = 0.0;
  last_processing_time_ms_ = 0.0;

  RCLCPP_INFO(this->get_logger(), "Node shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace wato::perception::bevfusion

RCLCPP_COMPONENTS_REGISTER_NODE(wato::perception::bevfusion::BEVFusionNode)
