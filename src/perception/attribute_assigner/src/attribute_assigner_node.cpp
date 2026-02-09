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

#include "attribute_assigner/attribute_assigner_node.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace wato::perception::attribute_assigner
{

AttributeAssignerNode::AttributeAssignerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("attribute_assigner_node", options)
, subscriber_qos_(10)
, publisher_qos_(10)
, last_stats_log_time_(std::chrono::steady_clock::now())
{
  RCLCPP_INFO(this->get_logger(), "Attribute Assigner ROS 2 lifecycle node created");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", this->get_current_state().label().c_str());
}

void AttributeAssignerNode::declareParameters(Params & params)
{
  // Class ID mappings (YOLOv8 COCO: 9=traffic_light, 2=car, 7=truck, 5=bus)
  this->declare_parameter<std::vector<std::string>>(
    "traffic_light_class_ids", std::vector<std::string>{"traffic_light", "9"});
  this->declare_parameter<std::vector<std::string>>(
    "car_class_ids", std::vector<std::string>{"car", "2", "truck", "7", "bus", "5"});
  this->declare_parameter<double>("min_detection_confidence", 0.3);

  params.traffic_light_class_ids = this->get_parameter("traffic_light_class_ids").as_string_array();
  params.car_class_ids = this->get_parameter("car_class_ids").as_string_array();
  params.min_detection_confidence = this->get_parameter("min_detection_confidence").as_double();

  // Traffic light color detection thresholds
  this->declare_parameter<double>("traffic_light_min_saturation", 60.0);
  this->declare_parameter<double>("traffic_light_min_value", 80.0);
  params.traffic_light_min_saturation = this->get_parameter("traffic_light_min_saturation").as_double();
  params.traffic_light_min_value = this->get_parameter("traffic_light_min_value").as_double();

  // Car signal detection thresholds (HSV-based)
  this->declare_parameter<double>("car_brake_min_brightness", 90.0);
  this->declare_parameter<double>("car_amber_hue_lo", 12.0);
  this->declare_parameter<double>("car_amber_hue_hi", 35.0);
  this->declare_parameter<double>("car_amber_min_saturation", 80.0);
  this->declare_parameter<double>("car_amber_min_value", 100.0);
  params.car_brake_min_brightness = this->get_parameter("car_brake_min_brightness").as_double();
  params.car_amber_hue_lo = this->get_parameter("car_amber_hue_lo").as_double();
  params.car_amber_hue_hi = this->get_parameter("car_amber_hue_hi").as_double();
  params.car_amber_min_saturation = this->get_parameter("car_amber_min_saturation").as_double();
  params.car_amber_min_value = this->get_parameter("car_amber_min_value").as_double();
}

void AttributeAssignerNode::syncedCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections_msg)
{
  if (!core_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Core not initialized; skipping");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat & image = cv_ptr->image;
  if (image.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Received empty image; passing detections through");
    if (detections_pub_ && detections_pub_->is_activated()) {
      detections_pub_->publish(*detections_msg);
    }
    return;
  }

  vision_msgs::msg::Detection2DArray enriched;
  try {
    enriched = core_->process(image, *detections_msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Error during attribute assignment: %s", e.what());
    return;
  }

  const double time_taken = core_->getLastProcessingTimeMs();
  updateStatistics(time_taken);

  if (detections_pub_ && detections_pub_->is_activated()) {
    detections_pub_->publish(enriched);
  }

  updateDiagnostics(detections_msg->header.stamp);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Processed %zu detections, %lu total. Time: %.3f ms",
    detections_msg->detections.size(),
    core_->getProcessedCount(),
    time_taken);
}

void AttributeAssignerNode::updateStatistics(double time_taken)
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

void AttributeAssignerNode::updateDiagnostics(const std_msgs::msg::Header::_stamp_type & timestamp)
{
  if (pub_diagnostic_) {
    pub_diagnostic_->tick(timestamp);
  }

  if (diagnostic_updater_) {
    diagnostic_updater_->force_update();
  }
}

void AttributeAssignerNode::logStatistics() const
{
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;

  RCLCPP_INFO(
    this->get_logger(), "Statistics: %lu arrays processed, average processing time: %.3f ms", processed, avg_time);
}

rclcpp::QoS AttributeAssignerNode::createSubscriberQoS(const std::string & reliability, int depth)
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

rclcpp::QoS AttributeAssignerNode::createPublisherQoS(
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

void AttributeAssignerNode::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const uint64_t processed = total_processed_.load();
  const double total_time = total_processing_time_ms_.load();
  const double avg_time = processed > 0 ? total_time / static_cast<double>(processed) : 0.0;
  const double last_time = last_processing_time_ms_.load();

  if (processed == 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No detection arrays processed yet");
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
  if (core_) {
    stat.add("Total Detections Processed", core_->getProcessedCount());
  }
}

// ---------------------------------------------------------------------------
// Lifecycle callbacks
// ---------------------------------------------------------------------------

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Attribute Assigner node");

  try {
    Params params;
    declareParameters(params);
    core_ = std::make_unique<AttributeAssignerCore>(params);

    // Log configured class IDs
    RCLCPP_INFO(this->get_logger(), "Traffic light class IDs: %zu configured", params.traffic_light_class_ids.size());
    RCLCPP_INFO(this->get_logger(), "Car/vehicle class IDs: %zu configured", params.car_class_ids.size());
    RCLCPP_INFO(this->get_logger(), "Min detection confidence: %.2f", params.min_detection_confidence);

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
    sync_queue_size_ = this->get_parameter("sync_queue_size").as_int();

    // Initialize diagnostics
    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("attribute_assigner");
    diagnostic_updater_->add("Attribute Assigner Status", this, &AttributeAssignerNode::diagnosticCallback);

    RCLCPP_INFO(this->get_logger(), "Node configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Configuration failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Attribute Assigner node");

  try {
    image_sub_ = std::make_unique<message_filters::Subscriber<ImageMsg, rclcpp_lifecycle::LifecycleNode>>(
      this, kImageTopic, subscriber_qos_.get_rmw_qos_profile());
    detections_sub_ = std::make_unique<message_filters::Subscriber<DetectionsMsg, rclcpp_lifecycle::LifecycleNode>>(
      this, kInputTopic, subscriber_qos_.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size_), *image_sub_, *detections_sub_);
    sync_->registerCallback(
      std::bind(&AttributeAssignerNode::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));

    detections_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(kOutputTopic, publisher_qos_);

    if (detections_pub_) {
      detections_pub_->on_activate();
    }

    if (diagnostic_updater_ && detections_pub_) {
      pub_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
        kOutputTopic,
        *diagnostic_updater_,
        diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10),
        diagnostic_updater::TimeStampStatusParam());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Node activated. Subscribed to image '%s' and detections '%s'; publishing -> '%s'",
      kImageTopic,
      kInputTopic,
      detections_pub_->get_topic_name());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Activation failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Attribute Assigner node");

  if (detections_pub_) {
    detections_pub_->on_deactivate();
  }

  sync_.reset();
  image_sub_.reset();
  detections_sub_.reset();

  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Attribute Assigner node");

  sync_.reset();
  image_sub_.reset();
  detections_sub_.reset();
  detections_pub_.reset();
  pub_diagnostic_.reset();
  diagnostic_updater_.reset();
  core_.reset();

  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    this->get_logger(), "Shutting down Attribute Assigner node from state: %s", previous_state.label().c_str());

  sync_.reset();
  image_sub_.reset();
  detections_sub_.reset();
  detections_pub_.reset();
  core_.reset();
  total_processed_ = 0;
  total_processing_time_ms_ = 0.0;
  last_processing_time_ms_ = 0.0;
  pub_diagnostic_.reset();
  diagnostic_updater_.reset();

  RCLCPP_INFO(this->get_logger(), "Node shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace wato::perception::attribute_assigner

RCLCPP_COMPONENTS_REGISTER_NODE(wato::perception::attribute_assigner::AttributeAssignerNode)
