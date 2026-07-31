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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/image_marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "bevfusion/bevfusion_core.hpp"

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
  // sync/QoS params are intentionally declared in on_configure() to avoid re-declaring already-registered parameters.

  // Directory containing all .plan and .onnx engine files for the model
  this->declare_parameter<std::string>("model_dir", "/opt/watonomous/models/bevfusion/resnet50");

  // Model precision
  this->declare_parameter<std::string>("precision", "int8");

  // Detection confidence, bounding boxes below threshold are discarded
  this->declare_parameter<double>("confidence_threshold", 0.3);

  // camera_names used by on_activate for TF lookups; num_cameras is derived from its size
  this->declare_parameter<std::vector<std::string>>(
    "camera_names",
    std::vector<std::string>{
      "camera_pano_nn", "camera_pano_ne", "camera_pano_nw", "camera_pano_ss", "camera_pano_se", "camera_pano_sw"});
  this->declare_parameter<int>("image_width", 1600);
  this->declare_parameter<int>("image_height", 900);
  this->declare_parameter<int>("norm_output_width", 704);
  this->declare_parameter<int>("norm_output_height", 256);

  // LiDAR voxelization parameters
  this->declare_parameter<std::vector<double>>("min_range", std::vector<double>{-54.0, -54.0, -5.0});
  this->declare_parameter<std::vector<double>>("max_range", std::vector<double>{54.0, 54.0, 3.0});
  this->declare_parameter<std::vector<double>>("voxel_size", std::vector<double>{0.075, 0.075, 0.2});
  this->declare_parameter<int>("max_points_per_voxel", 10);
  this->declare_parameter<int>("max_points", 300000);
  this->declare_parameter<int>("max_voxels", 160000);

  // BEV Fusion grid geometry
  this->declare_parameter<std::vector<double>>("xbound", std::vector<double>{-54.0, 54.0, 0.3});
  this->declare_parameter<std::vector<double>>("ybound", std::vector<double>{-54.0, 54.0, 0.3});
  this->declare_parameter<std::vector<double>>("zbound", std::vector<double>{-10.0, 10.0, 20.0});
  this->declare_parameter<std::vector<double>>("dbound", std::vector<double>{1.0, 60.0, 0.5});

  // Detection post-processing parameters
  this->declare_parameter<std::vector<double>>("post_center_range_start", std::vector<double>{-61.2, -61.2, -10.0});
  this->declare_parameter<std::vector<double>>("post_center_range_end", std::vector<double>{61.2, 61.2, 10.0});

  // Build BEVFusionInputConfig from declared parameters
  const std::string model_dir = this->get_parameter("model_dir").as_string();
  const auto camera_names = this->get_parameter("camera_names").as_string_array();

  // ROS params use double; BEVFusionInputConfig uses float — convert on read
  const auto to_float_vec = [this](const std::string & name) {
    const auto d = this->get_parameter(name).as_double_array();
    return std::vector<float>(d.begin(), d.end());
  };

  BEVFusionInputConfig config;

  config.camera_backbone_plan = model_dir + "/camera.backbone.plan";
  config.camera_vtransform_plan = model_dir + "/camera.vtransform.plan";
  config.fuser_plan = model_dir + "/fuser.plan";
  config.head_bbox_plan = model_dir + "/head.bbox.plan";
  config.lidar_backbone_onnx = model_dir + "/lidar.backbone.onnx";

  config.precision = this->get_parameter("precision").as_string();
  config.confidence_threshold = static_cast<float>(this->get_parameter("confidence_threshold").as_double());

  config.num_cameras = static_cast<int>(camera_names.size());
  config.image_width = this->get_parameter("image_width").as_int();
  config.image_height = this->get_parameter("image_height").as_int();
  config.norm_output_width = this->get_parameter("norm_output_width").as_int();
  config.norm_output_height = this->get_parameter("norm_output_height").as_int();

  config.min_range = to_float_vec("min_range");
  config.max_range = to_float_vec("max_range");
  config.voxel_size = to_float_vec("voxel_size");
  config.max_points_per_voxel = this->get_parameter("max_points_per_voxel").as_int();
  config.max_points = this->get_parameter("max_points").as_int();
  config.max_voxels = this->get_parameter("max_voxels").as_int();

  config.xbound = to_float_vec("xbound");
  config.ybound = to_float_vec("ybound");
  config.zbound = to_float_vec("zbound");
  config.dbound = to_float_vec("dbound");

  config.post_center_range_start = to_float_vec("post_center_range_start");
  config.post_center_range_end = to_float_vec("post_center_range_end");

  // transbbox_pc_range and transbbox_voxel_size are the XY projections of min_range and voxel_size
  config.transbbox_pc_range = {config.min_range[0], config.min_range[1]};
  config.transbbox_voxel_size = {config.voxel_size[0], config.voxel_size[1]};

  core_ = std::make_unique<BEVFusionCore>(config);
}

void BEVFusionNode::syncedCallback(
  const deep_msgs::msg::MultiImageCompressed::ConstSharedPtr & multi_image_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg)
{
  /**
   * TODO(bevfusion_team)
   * Synchronized callback — the main processing loop
   * When you receive a synced set of camera images + lidar:
   * 1. Extract camera images: Decode from CompressedImage → cv::Mat (BGR) → convert to RGB
   *    unsigned char* buffers (CUDA-BEVFusion expects RGB, see NormalizationParameter)
   * 2. Extract lidar points: Parse PointCloud2 → extract (x, y, z, intensity, ring) per
   *    point → flatten to std::vector<float> then convert to nvtype::half
   * 3. Call core_->infer(images, lidar_points, num_points)
   * 4. Convert results: BoundingBox → vision_msgs::Detection3DArray +
   *    visualization_msgs::MarkerArray
   * 5. Publish
   */

  multi_image_msg_count_++;
  lidar_msg_count_++;
  synced_msg_count_++;

  if (!core_ || !core_->initialize()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "[SYNC] BEVFusion Core not created or initialized; skipping");
    return;
  }

  if (!multi_image_msg || multi_image_msg->images.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "[SYNC] MultiImage message is null or empty; skipping");
    return;
  }

  if (!lidar_msg || lidar_msg->data.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "[SYNC] LiDAR point cloud message is null or empty; skipping");
    return;
  }

  const auto start = std::chrono::steady_clock::now();

  std::vector<const unsigned char *> camera_images;
  std::vector<cv::Mat> rgb_images;
  camera_images.reserve(multi_image_msg->images.size());
  rgb_images.reserve(multi_image_msg->images.size());

  for (size_t i = 0; i < multi_image_msg->images.size(); ++i) {
    const auto & frame_id = multi_image_msg->images[i].header.frame_id;
    cv::Mat bgr = decompressImage(multi_image_msg->images[i]);
    if (bgr.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Failed to decompress image for frame_id '%s'",
        multi_image_msg->images[i].header.frame_id.c_str());
      return;  // Skip this callback if any image fails to decompress
    }

    rgb_images.emplace_back();
    cv::cvtColor(bgr, rgb_images.back(), cv::COLOR_BGR2RGB);
    camera_images.push_back(rgb_images.back().data);
  }

  std::vector<float> lidar_data;

  processLidar(lidar_msg, lidar_data);

  // conversion to nvtype::half is done inside BEVFusionCore::infer, so we can just pass lidar_data as is.

  std::vector<BoundingBox> bboxes = core_->infer(camera_images, lidar_data, lidar_data.size() / 5);

  vision_msgs::msg::Detection3DArray detections_3d;
  visualization_msgs::msg::MarkerArray markers;

  for (const auto & bbox : bboxes) {
    detections_3d.detections.push_back(toDetection3D(bbox, multi_image_msg->header));
    markers.markers.push_back(toMarker(bbox, multi_image_msg->header, markers.markers.size()));
  }

  detection_pub_->publish(detections_3d);
  marker_pub_->publish(markers);

  const auto end = std::chrono::steady_clock::now();
  const double time_taken = std::chrono::duration<double, std::milli>(end - start).count();
  updateStatistics(time_taken);

  // updateDiagnostics();
}

void BEVFusionNode::multiCameraInfoCallback(
  const deep_msgs::msg::MultiCameraInfo::ConstSharedPtr & multi_camera_info_msg)
{
  if (multi_camera_info_msg->camera_infos.empty() || camera_info_received_) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received multi camera info with %zu cameras", multi_camera_info_msg->camera_infos.size());
  cached_multi_camera_info_ = multi_camera_info_msg;
  camera_info_received_ = true;
  computeCalibrationMatrices();
}

void BEVFusionNode::computeCalibrationMatrices()
{
  /**
   * TODO(bevfusion_team):
   * 1. Extract camera intrinsics K from cached_multi_camera_info_.
   * 2. Look up TF extrinsics (LiDAR <-> Camera frames) using tf_buffer_.
   * 3. Compute camera_to_lidar, camera_intrinsics, lidar_to_camera, and img_aug_matrix vectors.
   * 4. Call core_->updateCalibration(camera_to_lidar, camera_intrinsics, lidar_to_camera, img_aug_matrix).
   * 5. Set calibration_initialized_ = true.
   */

}

visualization_msgs::msg::Marker BEVFusionNode::toMarker(
  const BoundingBox & bbox, const std_msgs::msg::Header & header, int marker_id) const
{
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0; // default valid quaternion if no rotation set
  marker.pose.position.x = bbox.position.x;
  marker.pose.position.y = bbox.position.y;
  marker.pose.position.z = bbox.position.z;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, bbox.z_rotation);
  marker.pose.orientation = tf2::toMsg(q);

  // Set scale (size)
  marker.scale.x = bbox.size.l;
  marker.scale.y = bbox.size.w;
  marker.scale.z = bbox.size.h;
  // Color by class (e.g., cars=green, pedestrians=yellow, trucks=blue)
  switch (bbox.id) {
    case 0:  // Car
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      break;
    case 1:  // Pedestrian
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      break;
    case 2:  // Truck
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      break;
    default:
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 1.0f;
      break;
  }
  marker.lifetime = rclcpp::Duration(0, 100'000'000);  // 0.1s (so old markers disappear)
  marker.color.a = 0.8f;

  marker.header = header;
  marker.ns = "bevfusion_detections";
  marker.id = marker_id;

  return marker;
}

vision_msgs::msg::Detection3D BEVFusionNode::toDetection3D(const BoundingBox & bbox, const std_msgs::msg::Header & header) const
{
  vision_msgs::msg::Detection3D detection;

  detection.header = header;
  detection.bbox.center.position.x = bbox.position.x;
  detection.bbox.center.position.y = bbox.position.y;
  detection.bbox.center.position.z = bbox.position.z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, bbox.z_rotation);

  detection.bbox.center.orientation.x = q.x();
  detection.bbox.center.orientation.y = q.y();
  detection.bbox.center.orientation.z = q.z();
  detection.bbox.center.orientation.w = q.w();  

  detection.bbox.size.x = bbox.size.l;
  detection.bbox.size.y = bbox.size.w;
  detection.bbox.size.z = bbox.size.h;
  //(check axis convention — nuScenes uses l=forward, w=lateral, h=vertical)

  vision_msgs::msg::ObjectHypothesisWithPose hyp;
  hyp.hypothesis.class_id = std::to_string(bbox.id);
  hyp.hypothesis.score = bbox.score;
  detection.results.push_back(hyp);

  return detection;
}

// Copied from attribute assigner. I'll be honest I don't fully understrand this CV library syntax.
cv::Mat BEVFusionNode::decompressImage(const sensor_msgs::msg::CompressedImage & compressed_img) const
{
  try {
    cv::Mat bgr = cv::imdecode(cv::Mat(compressed_img.data), cv::IMREAD_COLOR);
    return bgr;  // Returns empty Mat on failure
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "OpenCV exception during decompression: %s", e.what());
    return cv::Mat();
  }
}

void BEVFusionNode::processLidar(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg, std::vector<float> & lidar_data)
{
  const size_t num_points = lidar_msg->width * lidar_msg->height;
  lidar_data.reserve(num_points * 5);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*lidar_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*lidar_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*lidar_msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*lidar_msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*lidar_msg, "ring");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_ring) {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;
    float intensity = *iter_intensity;
    uint16_t ring = *iter_ring;
    lidar_data.push_back(x);
    lidar_data.push_back(y);
    lidar_data.push_back(z);
    lidar_data.push_back(intensity);
    lidar_data.push_back(static_cast<float>(ring));
  }
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
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No ___TODO(bevfusion_team)____ processed yet");
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

    if (!core_) {
      throw std::runtime_error("BEVFusion core was not constructed from parameters");
    }

    if (!core_->initialize()) {
      throw std::runtime_error("Failed to initialize BEVFusion core");
    }

    // inside on_configure()
    if (!tf_buffer_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    }

    if (!tf_listener_) {
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

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
  /**
   * TODO(bevfusion_team)
   *
   * 1. [TODO] Check & Trigger Calibration:
   *    - Verify camera intrinsics are available (`cached_multi_camera_info_` / `camera_info_received_`).
   *    - Call `computeCalibrationMatrices()`, which looks up TF extrinsics, formats matrices,
   *      and invokes `core_->updateCalibration(...)`.
   *    - Ensure `calibration_initialized_ == true` before proceeding.
   *
   * 2. [DONE] Setup Message Synchronization:
   *    - Initialize `message_filters::Subscriber` instances for camera images and LiDAR point clouds.
   *    - Instantiate `message_filters::Synchronizer` with `ApproximateTime` policy.
   *    - Register `BEVFusionNode::syncedCallback` to handle synced pairs of (MultiImageCompressed, PointCloud2).
   *
   * 3. [DONE] Create Subscriptions & Publishers:
   *    - Subscriptions created for `multi_camera_info_sub_`, `lidar_sub_`, and `camera_sub_`.
   *    - Publishers instantiated for `detection_pub_` and `marker_pub_`.
   *
   * 4. [TODO] Logging
   *
   * 5. [TODO] Lifecycle Publisher Activation:
   *    - Call `detection_pub_->on_activate()` and `marker_pub_->on_activate()` to enable output publishing.
   */
  RCLCPP_INFO(this->get_logger(), "Activating BEVFusion node");
  RCLCPP_INFO(this->get_logger(), "Previous state: %s", prev_state.label().c_str());
  RCLCPP_INFO(this->get_logger(), "=============================================");

  try {
    RCLCPP_INFO(this->get_logger(), "Creating subscribers, publishers, and sync object");

    // Create subscribers
    multi_camera_info_sub_ = this->create_subscription<MultiCameraInfoMsg>(
      kCameraInfoTopic,
      subscriber_qos_,
      std::bind(&BEVFusionNode::multiCameraInfoCallback, this, std::placeholders::_1));

    multi_image_sub_ =
      std::make_shared<ImageSub>(this->shared_from_this(), kMultiImageTopic, subscriber_qos_.get_rmw_qos_profile());
    lidar_sub_ =
      std::make_shared<LidarSub>(this->shared_from_this(), kLidarTopic, subscriber_qos_.get_rmw_qos_profile());

    // ApproximateTime synchronizer
    // We use the message filters sync queue to synchronize the camera images and lidar data based on the timestamps.
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(sync_queue_size_), *multi_image_sub_, *lidar_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_max_time_diff_sec_));
    sync_->registerCallback(
      std::bind(&BEVFusionNode::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Create publishers
    detection_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(kOutputDetectionsTopic, publisher_qos_);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kOutputMarkersTopic, publisher_qos_);

    RCLCPP_INFO(this->get_logger(), "=============================================");
    RCLCPP_INFO(this->get_logger(), "Node activated successfully!");
    RCLCPP_INFO(this->get_logger(), "Subscribed to:");
    // TODO(bevfusion_team)
    RCLCPP_INFO(this->get_logger(), "Publishing to:");
    // TODO(bevfusion_team)
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

  // TODO(bevfusion_team)

  RCLCPP_INFO(this->get_logger(), "Cleaning up publishers...");
  // TODO(bevfusion_team)
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
  // TODO(bevfusion_team)

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
