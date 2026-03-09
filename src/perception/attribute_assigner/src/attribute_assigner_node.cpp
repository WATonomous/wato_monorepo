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
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

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
  const deep_msgs::msg::MultiImageCompressed::ConstSharedPtr & multi_image_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections_msg)
{
  if (!core_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Core not initialized; skipping");
    return;
  }

  if (!multi_image_msg || multi_image_msg->images.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Received empty MultiImage; passing %zu detections through", detections_msg->detections.size());
    if (detections_pub_ && detections_pub_->is_activated()) {
      detections_pub_->publish(*detections_msg);
    }
    return;
  }

  const auto start = std::chrono::steady_clock::now();

  // Build frame_id -> MultiImage index lookup for O(1) access
  std::unordered_map<std::string, size_t> frame_id_to_index;
  for (size_t i = 0; i < multi_image_msg->images.size(); ++i) {
    frame_id_to_index[multi_image_msg->images[i].header.frame_id] = i;
  }

  // First pass: collect unique frame_ids that have traffic lights or cars
  std::unordered_set<std::string> frames_to_decompress;
  for (const auto & det : detections_msg->detections) {
    if (!core_->isTrafficLight(det) && !core_->isCar(det)) {
      continue;
    }
    const double score = core_->getBestScore(det);
    if (score < core_->getParams().min_detection_confidence) {
      continue;
    }
    // Use detection's header.frame_id to identify which camera this came from
    if (!det.header.frame_id.empty() && frame_id_to_index.count(det.header.frame_id) > 0) {
      frames_to_decompress.insert(det.header.frame_id);
    }
  }

  // If no frames to decompress, pass through
  if (frames_to_decompress.empty()) {
    if (detections_pub_ && detections_pub_->is_activated()) {
      detections_pub_->publish(*detections_msg);
    }
    return;
  }

  // Decompress only the needed images (once per frame)
  std::unordered_map<std::string, cv::Mat> decompressed_images;
  for (const auto & frame_id : frames_to_decompress) {
    const size_t idx = frame_id_to_index[frame_id];
    cv::Mat image = decompressImage(multi_image_msg->images[idx]);
    if (!image.empty()) {
      decompressed_images[frame_id] = std::move(image);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Failed to decompress image for frame_id '%s'", frame_id.c_str());
    }
  }

  // Process detections
  vision_msgs::msg::Detection2DArray enriched;
  enriched.header = detections_msg->header;
  enriched.detections.reserve(detections_msg->detections.size());

  for (const auto & det : detections_msg->detections) {
    vision_msgs::msg::Detection2D enriched_det = det;

    // Check if this detection is a traffic light or car
    const bool is_traffic_light = core_->isTrafficLight(det);
    const bool is_car = core_->isCar(det);
    if (!is_traffic_light && !is_car) {
      enriched.detections.push_back(enriched_det);
      continue;
    }

    const double score = core_->getBestScore(det);
    if (score < core_->getParams().min_detection_confidence) {
      enriched.detections.push_back(enriched_det);
      continue;
    }

    // Look up the decompressed image by detection's frame_id
    const std::string& frame_id = det.header.frame_id;
    auto img_it = decompressed_images.find(frame_id);
    if (img_it == decompressed_images.end() || img_it->second.empty()) {
      enriched.detections.push_back(enriched_det);
      continue;
    }

    // Crop to detection bbox
    cv::Mat crop = core_->cropToBbox(img_it->second, det);
    if (crop.empty()) {
      enriched.detections.push_back(enriched_det);
      continue;
    }

    // Classify and append attributes
    if (is_traffic_light) {
      auto attrs = core_->classifyTrafficLightState(crop);
      core_->appendTrafficLightHypotheses(enriched_det, attrs);
    } else if (is_car) {
      auto attrs = core_->classifyCarBehavior(crop);
      core_->appendCarHypotheses(enriched_det, attrs);
    }

    enriched.detections.push_back(enriched_det);
  }

  const auto end = std::chrono::steady_clock::now();
  const double time_taken = std::chrono::duration<double, std::milli>(end - start).count();
  updateStatistics(time_taken);

  if (detections_pub_ && detections_pub_->is_activated()) {
    detections_pub_->publish(enriched);
  }

  updateDiagnostics(detections_msg->header.stamp);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Processed %zu detections (%zu frames decompressed). Time: %.3f ms",
    detections_msg->detections.size(),
    decompressed_images.size(),
    time_taken);
}

cv::Mat AttributeAssignerNode::decompressImage(const sensor_msgs::msg::CompressedImage & compressed_img) const
{
  try {
    cv::Mat image = cv::imdecode(cv::Mat(compressed_img.data), cv::IMREAD_COLOR);
    return image;  // Returns empty Mat on failure
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "OpenCV exception during decompression: %s", e.what());
    return cv::Mat();
  }
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
    // Synchronize MultiImage with Detection2DArray
    multi_image_sub_ = std::make_unique<message_filters::Subscriber<MultiImageMsg, rclcpp_lifecycle::LifecycleNode>>(
      this, kMultiImageTopic, subscriber_qos_.get_rmw_qos_profile());
    detections_sub_ = std::make_unique<message_filters::Subscriber<DetectionsMsg, rclcpp_lifecycle::LifecycleNode>>(
      this, kInputTopic, subscriber_qos_.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size_), *multi_image_sub_, *detections_sub_);
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
      "Node activated. Subscribed to MultiImage '%s' and detections '%s'; publishing -> '%s'",
      kMultiImageTopic,
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
  multi_image_sub_.reset();
  detections_sub_.reset();

  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AttributeAssignerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Attribute Assigner node");

  sync_.reset();
  multi_image_sub_.reset();
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
  multi_image_sub_.reset();
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
