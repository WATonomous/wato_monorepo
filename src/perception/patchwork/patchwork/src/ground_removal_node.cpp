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

#include "patchworkpp/ground_removal_node.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace wato::perception::patchworkpp
{

GroundRemovalNode::GroundRemovalNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("patchworkpp_node", options)
, subscriber_qos_(10)
, publisher_qos_(10)
, last_stats_log_time_(std::chrono::steady_clock::now())
{
  RCLCPP_INFO(this->get_logger(), "Patchwork++ Ground Removal ROS 2 lifecycle node created");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", this->get_current_state().label().c_str());
}

Eigen::MatrixX3f GroundRemovalNode::filterInvalidPoints(const Eigen::MatrixX3f & cloud)
{
  size_t valid_points = 0;
  Eigen::MatrixX3f filtered_cloud(cloud.rows(), 3);

  for (int i = 0; i < cloud.rows(); ++i) {
    if (std::isfinite(cloud(i, 0)) && std::isfinite(cloud(i, 1)) && std::isfinite(cloud(i, 2))) {
      filtered_cloud.row(valid_points++) = cloud.row(i);
    }
  }

  if (valid_points == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "All points invalid after filtering: received %zu points, but all contained NaN/Inf values. Skipping processing.",
      static_cast<size_t>(cloud.rows()));
    return Eigen::MatrixX3f(0, 3);
  }

  if (valid_points < static_cast<size_t>(cloud.rows())) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Filtered %zu invalid points (NaN/Inf) from cloud, processing %zu valid points",
      static_cast<size_t>(cloud.rows()) - valid_points,
      valid_points);
    filtered_cloud.conservativeResize(static_cast<int>(valid_points), 3);
  }

  return filtered_cloud;
}

void GroundRemovalNode::updateStatistics(double time_taken)
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

void GroundRemovalNode::updateDiagnostics(const std_msgs::msg::Header::_stamp_type & timestamp)
{
  if (ground_pub_diagnostic_ && nonground_pub_diagnostic_) {
    ground_pub_diagnostic_->tick(timestamp);
    nonground_pub_diagnostic_->tick(timestamp);
  }

  if (diagnostic_updater_) {
    diagnostic_updater_->force_update();
  }
}

void GroundRemovalNode::declareParameters(patchwork::Params & params)
{
  const auto declare_if_missing = [this](const std::string & name, const auto & default_value) {
    using ValueType = std::decay_t<decltype(default_value)>;
    if (!this->has_parameter(name)) {
      this->declare_parameter<ValueType>(name, default_value);
    }
    return this->get_parameter(name).template get_value<ValueType>();
  };

  params.sensor_height = declare_if_missing("sensor_height", 1.88);
  params.num_iter = declare_if_missing("num_iter", 3);
  params.num_lpr = declare_if_missing("num_lpr", 20);
  params.num_min_pts = declare_if_missing("num_min_pts", 0);
  params.th_seeds = declare_if_missing("th_seeds", 0.3);
  params.th_dist = declare_if_missing("th_dist", 0.10);
  params.th_seeds_v = declare_if_missing("th_seeds_v", 0.25);
  params.th_dist_v = declare_if_missing("th_dist_v", 0.85);
  params.max_range = declare_if_missing("max_range", 80.0);
  params.min_range = declare_if_missing("min_range", 1.0);
  params.uprightness_thr = declare_if_missing("uprightness_thr", 0.101);
  params.enable_RNR = declare_if_missing("enable_RNR", false);
  params.verbose = declare_if_missing("verbose", true);
}

void GroundRemovalNode::removeGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  Eigen::MatrixX3f cloud;
  try {
    cloud = GroundRemovalCore::pointCloud2ToEigen(msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Failed to convert PointCloud2 to Eigen: %s", e.what());
    return;
  }

  if (cloud.rows() == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Received empty point cloud (0 points) from topic '%s', skipping processing",
      pointcloud_sub_ ? pointcloud_sub_->get_topic_name() : kCloudTopic);
    return;
  }

  Eigen::MatrixX3f filtered_cloud = filterInvalidPoints(cloud);
  if (filtered_cloud.rows() == 0) {
    return;
  }

  try {
    core_->process(filtered_cloud);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Error during ground removal processing: %s", e.what());
    return;
  }

  Eigen::MatrixX3f ground = core_->getGround();
  Eigen::MatrixX3f nonground = core_->getNonground();
  const double time_taken = core_->getTimeTaken();

  updateStatistics(time_taken);
  publishSegments(ground, nonground, msg->header);
  updateDiagnostics(msg->header.stamp);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Processed %zu points (after filtering): %zu ground, %zu non-ground (removed). Time: %.3f ms",
    static_cast<size_t>(filtered_cloud.rows()),
    static_cast<size_t>(ground.rows()),
    static_cast<size_t>(nonground.rows()),
    time_taken);
}

void GroundRemovalNode::publishSegments(
  const Eigen::MatrixX3f & ground_points,
  const Eigen::MatrixX3f & nonground_points,
  const std_msgs::msg::Header & in_header)
{
  if (!ground_publisher_ || !nonground_publisher_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Publishers not ready; skipping ground segmentation publish");
    return;
  }

  if (ground_publisher_->is_activated() && nonground_publisher_->is_activated()) {
    ground_publisher_->publish(GroundRemovalCore::eigenToPointCloud2(ground_points, in_header));
    nonground_publisher_->publish(GroundRemovalCore::eigenToPointCloud2(nonground_points, in_header));
  } else {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Publishers not activated; skipping ground segmentation publish");
  }
}

void GroundRemovalNode::logStatistics() const
{
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;

  RCLCPP_INFO(
    this->get_logger(), "Statistics: %lu clouds processed, average processing time: %.3f ms", processed, avg_time);
}

rclcpp::QoS GroundRemovalNode::createSubscriberQoS(const std::string & reliability, int depth)
{
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

rclcpp::QoS GroundRemovalNode::createPublisherQoS(
  const std::string & reliability, const std::string & durability, int depth)
{
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

void GroundRemovalNode::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;
  const double last_time = last_processing_time_ms_.load();

  if (processed == 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No clouds processed yet");
  } else if (avg_time > 100.0) {  // If average processing takes >100ms
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "High processing latency");
  } else if (last_time > 200.0) {  // If last processing took >200ms
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Recent high processing latency");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Operating normally");
  }

  stat.add("Clouds Processed", processed);
  stat.add("Average Processing Time (ms)", avg_time);
  stat.add("Last Processing Time (ms)", last_time);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GroundRemovalNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Patchwork++ Ground Removal node");

  try {
    patchwork::Params params;
    declareParameters(params);
    core_ = std::make_unique<GroundRemovalCore>(params);

    const auto declare_if_missing = [this](const std::string & name, const auto & default_value) {
      using ValueType = std::decay_t<decltype(default_value)>;
      if (!this->has_parameter(name)) {
        this->declare_parameter<ValueType>(name, default_value);
      }
    };

    declare_if_missing("qos_subscriber_reliability", std::string("best_effort"));
    declare_if_missing("qos_subscriber_depth", 10);
    declare_if_missing("qos_publisher_reliability", std::string("reliable"));
    declare_if_missing("qos_publisher_durability", std::string("transient_local"));
    declare_if_missing("qos_publisher_depth", 10);

    const std::string subscriber_reliability = this->get_parameter("qos_subscriber_reliability").as_string();
    const int subscriber_depth = this->get_parameter("qos_subscriber_depth").as_int();
    subscriber_qos_ = createSubscriberQoS(subscriber_reliability, subscriber_depth);

    const std::string publisher_reliability = this->get_parameter("qos_publisher_reliability").as_string();
    const std::string publisher_durability = this->get_parameter("qos_publisher_durability").as_string();
    const int publisher_depth = this->get_parameter("qos_publisher_depth").as_int();
    publisher_qos_ = createPublisherQoS(publisher_reliability, publisher_durability, publisher_depth);

    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("patchworkpp");
    diagnostic_updater_->add("Ground Removal Status", this, &GroundRemovalNode::diagnosticCallback);

    RCLCPP_INFO(this->get_logger(), "Node configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Configuration failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GroundRemovalNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Patchwork++ Ground Removal node");

  try {
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      kCloudTopic, subscriber_qos_, std::bind(&GroundRemovalNode::removeGround, this, std::placeholders::_1));

    ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(kGroundTopic, publisher_qos_);
    nonground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(kNonGroundTopic, publisher_qos_);

    if (ground_publisher_) {
      ground_publisher_->on_activate();
    }
    if (nonground_publisher_) {
      nonground_publisher_->on_activate();
    }

    if (diagnostic_updater_ && ground_publisher_ && nonground_publisher_) {
      ground_pub_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
        kGroundTopic,
        *diagnostic_updater_,
        diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10),
        diagnostic_updater::TimeStampStatusParam());
      nonground_pub_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
        kNonGroundTopic,
        *diagnostic_updater_,
        diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10),
        diagnostic_updater::TimeStampStatusParam());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Node activated. Subscribed to '%s'; publishing ground → '%s', non-ground → '%s'",
      pointcloud_sub_->get_topic_name(),
      ground_publisher_->get_topic_name(),
      nonground_publisher_->get_topic_name());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Activation failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GroundRemovalNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Patchwork++ Ground Removal node");

  if (ground_publisher_) {
    ground_publisher_->on_deactivate();
  }
  if (nonground_publisher_) {
    nonground_publisher_->on_deactivate();
  }

  pointcloud_sub_.reset();

  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GroundRemovalNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Patchwork++ Ground Removal node");

  pointcloud_sub_.reset();
  ground_publisher_.reset();
  nonground_publisher_.reset();
  ground_pub_diagnostic_.reset();
  nonground_pub_diagnostic_.reset();
  diagnostic_updater_.reset();
  core_.reset();

  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GroundRemovalNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    this->get_logger(), "Shutting down Patchwork++ Ground Removal node from state: %s", previous_state.label().c_str());

  pointcloud_sub_.reset();
  ground_publisher_.reset();
  nonground_publisher_.reset();
  core_.reset();
  total_processed_ = 0;
  total_processing_time_ms_ = 0.0;
  last_processing_time_ms_ = 0.0;
  ground_pub_diagnostic_.reset();
  nonground_pub_diagnostic_.reset();
  diagnostic_updater_.reset();

  RCLCPP_INFO(this->get_logger(), "Node shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace wato::perception::patchworkpp

RCLCPP_COMPONENTS_REGISTER_NODE(wato::perception::patchworkpp::GroundRemovalNode)
