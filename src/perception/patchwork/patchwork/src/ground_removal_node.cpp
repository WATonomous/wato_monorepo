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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace wato::perception::patchworkpp
{

GroundRemovalNode::GroundRemovalNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("patchworkpp_node", options),
  subscriber_qos_(10),
  publisher_qos_(10),
  last_stats_log_time_(std::chrono::steady_clock::now()),
  point_cloud_limits_(PointCloudLimits::defaultLimits())
{
  this->declare_parameter<double>("sensor_height", 1.88);
  this->declare_parameter<int>("num_iter", 3);
  this->declare_parameter<int>("num_lpr", 20);
  this->declare_parameter<int>("num_min_pts", 0);
  this->declare_parameter<double>("th_seeds", 0.3);
  this->declare_parameter<double>("th_dist", 0.10);
  this->declare_parameter<double>("th_seeds_v", 0.25);
  this->declare_parameter<double>("th_dist_v", 0.85);
  this->declare_parameter<double>("max_range", 80.0);
  this->declare_parameter<double>("min_range", 1.0);
  this->declare_parameter<double>("uprightness_thr", 0.101);
  this->declare_parameter<bool>("enable_RNR", false);
  this->declare_parameter<bool>("verbose", true);
  this->declare_parameter<std::string>("qos_subscriber_reliability", "best_effort");
  this->declare_parameter<int>("qos_subscriber_depth", 10);
  this->declare_parameter<std::string>("qos_publisher_reliability", "reliable");
  this->declare_parameter<std::string>("qos_publisher_durability", "transient_local");
  this->declare_parameter<int>("qos_publisher_depth", 10);
  this->declare_parameter<int64_t>("point_cloud_limits.max_points", static_cast<int64_t>(PointCloudLimits::defaultLimits().max_points));
  this->declare_parameter<int64_t>("point_cloud_limits.min_points", static_cast<int64_t>(PointCloudLimits::defaultLimits().min_points));

  RCLCPP_INFO(this->get_logger(), "Patchwork++ Ground Removal ROS 2 lifecycle node created");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", this->get_current_state().label().c_str());
}

bool GroundRemovalNode::validatePointCloudDimensions(size_t num_points) const
{
  if (num_points < point_cloud_limits_.min_points) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Point cloud validation failed: received %zu points, but minimum required is %zu points. "
      "Skipping processing. Consider adjusting 'point_cloud_limits.min_points' if this is expected.",
      num_points,
      point_cloud_limits_.min_points);
    return false;
  }
  if (num_points > point_cloud_limits_.max_points) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Point cloud validation failed: received %zu points, but maximum allowed is %zu points "
      "(exceeds limit by %zu points). Skipping processing. "
      "Consider increasing 'point_cloud_limits.max_points' if your LiDAR produces larger point clouds.",
      num_points,
      point_cloud_limits_.max_points,
      num_points - point_cloud_limits_.max_points);
    return false;
  }
  return true;
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
  params.sensor_height = this->get_parameter("sensor_height").as_double();
  params.num_iter = this->get_parameter("num_iter").as_int();
  params.num_lpr = this->get_parameter("num_lpr").as_int();
  params.num_min_pts = this->get_parameter("num_min_pts").as_int();
  params.th_seeds = this->get_parameter("th_seeds").as_double();
  params.th_dist = this->get_parameter("th_dist").as_double();
  params.th_seeds_v = this->get_parameter("th_seeds_v").as_double();
  params.th_dist_v = this->get_parameter("th_dist_v").as_double();
  params.max_range = this->get_parameter("max_range").as_double();
  params.min_range = this->get_parameter("min_range").as_double();
  params.uprightness_thr = this->get_parameter("uprightness_thr").as_double();
  params.enable_RNR = this->get_parameter("enable_RNR").as_bool();
  params.verbose = this->get_parameter("verbose").as_bool();
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

  const size_t num_points = static_cast<size_t>(cloud.rows());
  if (!validatePointCloudDimensions(num_points)) {
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
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Publishers not activated; skipping ground segmentation publish");
  }
}

void GroundRemovalNode::logStatistics() const
{
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;

  RCLCPP_INFO(
    this->get_logger(),
    "Statistics: %lu clouds processed, average processing time: %.3f ms",
    processed,
    avg_time);
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

    const std::string subscriber_reliability = this->get_parameter("qos_subscriber_reliability").as_string();
    const int subscriber_depth = this->get_parameter("qos_subscriber_depth").as_int();
    subscriber_qos_ = createSubscriberQoS(subscriber_reliability, subscriber_depth);

    const std::string publisher_reliability = this->get_parameter("qos_publisher_reliability").as_string();
    const std::string publisher_durability = this->get_parameter("qos_publisher_durability").as_string();
    const int publisher_depth = this->get_parameter("qos_publisher_depth").as_int();
    publisher_qos_ = createPublisherQoS(publisher_reliability, publisher_durability, publisher_depth);

    point_cloud_limits_.max_points = static_cast<size_t>(this->get_parameter("point_cloud_limits.max_points").as_int());
    point_cloud_limits_.min_points = static_cast<size_t>(this->get_parameter("point_cloud_limits.min_points").as_int());
    RCLCPP_INFO(
      this->get_logger(),
      "Point cloud limits configured: min=%zu, max=%zu points",
      point_cloud_limits_.min_points,
      point_cloud_limits_.max_points);

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
        kGroundTopic, *diagnostic_updater_, diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10),
        diagnostic_updater::TimeStampStatusParam());
      nonground_pub_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
        kNonGroundTopic, *diagnostic_updater_,
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
    this->get_logger(),
    "Shutting down Patchwork++ Ground Removal node from state: %s",
    previous_state.label().c_str());

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<wato::perception::patchworkpp::GroundRemovalNode>(rclcpp::NodeOptions());
  exec.add_node(node->get_node_base_interface());

  auto configured_state = node->configure();
  if (configured_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    auto activated_state = node->activate();
    if (activated_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(node->get_logger(), "Node configured and activated, spinning...");
      exec.spin();
    } else {
      RCLCPP_ERROR(
        node->get_logger(), "Failed to activate node, current state: %s", activated_state.label().c_str());
    }
  } else {
    RCLCPP_ERROR(
      node->get_logger(), "Failed to configure node, current state: %s", configured_state.label().c_str());
  }

  rclcpp::shutdown();
  return 0;
}