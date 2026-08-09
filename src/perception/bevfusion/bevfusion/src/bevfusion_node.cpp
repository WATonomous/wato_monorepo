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
#include <Eigen/Core>
#include <Eigen/Dense>
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
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/image_marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

  // Frame IDs
  this->declare_parameter<std::string>("lidar_frame_id", "lidar_cc");
  lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();

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
  this->declare_parameter<int>("image_width", 1280);
  this->declare_parameter<int>("image_height", 1024);
  this->declare_parameter<float>("resize_lim", 0.55f);
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
  camera_names_ = this->get_parameter("camera_names").as_string_array();

  // ROS params use double; BEVFusionInputConfig uses float — convert on read
  const auto to_float_vec = [this](const std::string & name) {
    const auto d = this->get_parameter(name).as_double_array();
    return std::vector<float>(d.begin(), d.end());
  };

  config_.camera_backbone_plan = model_dir + "/camera.backbone.plan";
  config_.camera_vtransform_plan = model_dir + "/camera.vtransform.plan";
  config_.fuser_plan = model_dir + "/fuser.plan";
  config_.head_bbox_plan = model_dir + "/head.bbox.plan";
  config_.lidar_backbone_onnx = model_dir + "/lidar.backbone.xyz.onnx";

  config_.precision = this->get_parameter("precision").as_string();
  config_.confidence_threshold = static_cast<float>(this->get_parameter("confidence_threshold").as_double());

  config_.num_cameras = static_cast<int>(camera_names_.size());
  config_.image_width = this->get_parameter("image_width").as_int();
  config_.image_height = this->get_parameter("image_height").as_int();
  config_.resize_lim = static_cast<float>(this->get_parameter("resize_lim").as_double());
  config_.norm_output_width = this->get_parameter("norm_output_width").as_int();
  config_.norm_output_height = this->get_parameter("norm_output_height").as_int();

  config_.min_range = to_float_vec("min_range");
  config_.max_range = to_float_vec("max_range");
  config_.voxel_size = to_float_vec("voxel_size");
  config_.max_points_per_voxel = this->get_parameter("max_points_per_voxel").as_int();
  config_.max_points = this->get_parameter("max_points").as_int();
  config_.max_voxels = this->get_parameter("max_voxels").as_int();

  config_.xbound = to_float_vec("xbound");
  config_.ybound = to_float_vec("ybound");
  config_.zbound = to_float_vec("zbound");
  config_.dbound = to_float_vec("dbound");

  config_.post_center_range_start = to_float_vec("post_center_range_start");
  config_.post_center_range_end = to_float_vec("post_center_range_end");

  // transbbox_pc_range and transbbox_voxel_size are the XY projections of min_range and voxel_size
  config_.transbbox_pc_range = {config_.min_range[0], config_.min_range[1]};
  config_.transbbox_voxel_size = {config_.voxel_size[0], config_.voxel_size[1]};

  core_ = std::make_unique<BEVFusionCore>(config_);
}

void BEVFusionNode::syncedCallback(
  const deep_msgs::msg::MultiImageCompressed::ConstSharedPtr & multi_image_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg)
{
  RCLCPP_INFO(this->get_logger(), "[SYNC] Synced callback called");
  multi_image_msg_count_++;
  lidar_msg_count_++;
  synced_msg_count_++;

  if (!core_ || !core_->initialize()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "[SYNC] BEVFusion Core not created or initialized; skipping");
    return;
  }

  if (!calibration_initialized_.load()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "[SYNC] Calibration not initialized; skipping");
    return;
  }

  if (!multi_image_msg || multi_image_msg->images.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "[SYNC] MultiImage message is null or empty; skipping");
    return;
  }

  if (!lidar_msg || lidar_msg->data.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "[SYNC] LiDAR point cloud message is null or empty; skipping");
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
        this->get_logger(), *this->get_clock(), 5000, "Failed to decompress image for frame_id '%s'", frame_id.c_str());
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

  updateDiagnostics(detections_3d.header.stamp);
}

void BEVFusionNode::multiCameraInfoCallback(
  const deep_msgs::msg::MultiCameraInfo::ConstSharedPtr & multi_camera_info_msg)
{
  if (multi_camera_info_msg->camera_infos.empty() || calibration_initialized_) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received multi camera info with %zu cameras", multi_camera_info_msg->camera_infos.size());

  // Set camera info only for the cameras that are present in the list camera_names_
  // And ensure they are in the same order as camera_names_
  MultiCameraInfoMsg::SharedPtr filtered_multi_camera_info_msg = std::make_shared<MultiCameraInfoMsg>();
  filtered_multi_camera_info_msg->camera_infos.reserve(camera_names_.size());
  for (const auto & camera_name : camera_names_) {
    for (const auto & camera_info : multi_camera_info_msg->camera_infos) {
      if (camera_info.header.frame_id == camera_name) {
        filtered_multi_camera_info_msg->camera_infos.push_back(camera_info);
        break;
      }
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Filtered multi camera info has %zu cameras",
    filtered_multi_camera_info_msg->camera_infos.size());

  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    cached_multi_camera_info_ = filtered_multi_camera_info_msg;
  }
  computeCalibrationMatrices();
}

void BEVFusionNode::computeCalibrationMatrices()
{
  // Get cached MultiCameraInfoMsg
  MultiCameraInfoMsg::ConstSharedPtr multi_camera_info;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    multi_camera_info = cached_multi_camera_info_;
  }

  if (!multi_camera_info || multi_camera_info->camera_infos.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot compute calibration: no cached MultiCameraInfo");
    return;
  }

  // Create flat matrices to store calibration data.
  // NOTE: BEVFusion expects matrices in row-major order.
  std::vector<float> camera_to_lidar;
  std::vector<float> camera_intrinsics;
  std::vector<float> lidar_to_image_projection;
  std::vector<float> img_aug_matrix;
  camera_to_lidar.reserve(multi_camera_info->camera_infos.size() * 16);
  camera_intrinsics.reserve(multi_camera_info->camera_infos.size() * 16);
  lidar_to_image_projection.reserve(multi_camera_info->camera_infos.size() * 16);
  img_aug_matrix.reserve(multi_camera_info->camera_infos.size() * 16);

  // For each camera in the MultiCameraInfoMsg, extract the data
  for (const auto & camera_info : multi_camera_info->camera_infos) {
    // Extract camera intrinsics:
    // - Pad the 3x3 K matrix to 4x4 with 0s and a 1 in the bottom-right corner.
    // - cam_intrinsic is a flat matrix
    // - Auto-formatting is not great, but this should look like a 4x4 matrix
    const auto & k = camera_info.k;
    float cam_intrinsic[16] = {
      static_cast<float>(k[0]),
      static_cast<float>(k[1]),
      static_cast<float>(k[2]),
      0.0f,
      static_cast<float>(k[3]),
      static_cast<float>(k[4]),
      static_cast<float>(k[5]),
      0.0f,
      static_cast<float>(k[6]),
      static_cast<float>(k[7]),
      static_cast<float>(k[8]),
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      1.0f};
    camera_intrinsics.insert(camera_intrinsics.end(), std::begin(cam_intrinsic), std::end(cam_intrinsic));

    // Extract camera to lidar (combined lidar transform) extrinsics from TF:
    // 1. Lookup transform
    // 2. Convert ROS2 TransformStamped to an Eigen::Isometry3d
    // 3. Cast to float matrix (BEVFusion typically expects 32-bit floats)
    // 4. Flatten and insert the 4x4 matrix into a row-major 16-element vector
    geometry_msgs::msg::TransformStamped cam2lidar_tf;
    try {
      cam2lidar_tf = tf_buffer_->lookupTransform(lidar_frame_id_, camera_info.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }
    Eigen::Isometry3d eigen_transform = tf2::transformToEigen(cam2lidar_tf);
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> cam2lidar_matrix = eigen_transform.matrix().cast<float>();
    camera_to_lidar.insert(camera_to_lidar.end(), cam2lidar_matrix.data(), cam2lidar_matrix.data() + 16);

    // Extract the lidar to image projection matrix
    // - lidar2image = K * inverse(cam2lidar)
    // - K_matrix is the matrix version of the flat cam_intrinsic
    Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> K_matrix(cam_intrinsic);
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lidar2cam = cam2lidar_matrix.inverse();
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lidar2img = K_matrix * lidar2cam;
    lidar_to_image_projection.insert(lidar_to_image_projection.end(), lidar2img.data(), lidar2img.data() + 16);

    // Create image aug matrix
    // - Scale: resize_lim
    // - X Translation: -crop_x
    // - Y Translation: -crop_y
    int resized_w = static_cast<int>(config_.image_width * config_.resize_lim);
    int resized_h = static_cast<int>(config_.image_height * config_.resize_lim);
    int crop_x = (resized_w - config_.norm_output_width) / 2;
    int crop_y = resized_h - config_.norm_output_height;
    // - Auto-formatting is not great, but this should look like a 4x4 matrix
    float aug[16] = {
      config_.resize_lim,
      0.0f,
      static_cast<float>(-crop_x),
      0.0f,
      0.0f,
      config_.resize_lim,
      static_cast<float>(-crop_y),
      0.0f,
      0.0f,
      0.0f,
      1.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      1.0f};
    img_aug_matrix.insert(img_aug_matrix.end(), std::begin(aug), std::end(aug));
  }

  // Log matrix values neatly
  RCLCPP_INFO(this->get_logger(), "Camera intrinsics: ");
  for (size_t i = 0; i < camera_intrinsics.size(); i += 4) {
    RCLCPP_INFO(
      this->get_logger(),
      "[%f %f %f %f]",
      camera_intrinsics[i],
      camera_intrinsics[i + 1],
      camera_intrinsics[i + 2],
      camera_intrinsics[i + 3]);
  }
  RCLCPP_INFO(this->get_logger(), "Camera to lidar: ");
  for (size_t i = 0; i < camera_to_lidar.size(); i += 4) {
    RCLCPP_INFO(
      this->get_logger(),
      "[%f %f %f %f]",
      camera_to_lidar[i],
      camera_to_lidar[i + 1],
      camera_to_lidar[i + 2],
      camera_to_lidar[i + 3]);
  }
  RCLCPP_INFO(this->get_logger(), "Lidar to image projection: ");
  for (size_t i = 0; i < lidar_to_image_projection.size(); i += 4) {
    RCLCPP_INFO(
      this->get_logger(),
      "[%f %f %f %f]",
      lidar_to_image_projection[i],
      lidar_to_image_projection[i + 1],
      lidar_to_image_projection[i + 2],
      lidar_to_image_projection[i + 3]);
  }
  RCLCPP_INFO(this->get_logger(), "Image aug matrix: ");
  for (size_t i = 0; i < img_aug_matrix.size(); i += 4) {
    RCLCPP_INFO(
      this->get_logger(),
      "[%f %f %f %f]",
      img_aug_matrix[i],
      img_aug_matrix[i + 1],
      img_aug_matrix[i + 2],
      img_aug_matrix[i + 3]);
  }

  // Send to core
  core_->updateCalibration(camera_to_lidar, camera_intrinsics, lidar_to_image_projection, img_aug_matrix);

  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    calibration_initialized_ = true;
  }
  RCLCPP_INFO(this->get_logger(), "Calibration computed successfully");
}

visualization_msgs::msg::Marker BEVFusionNode::toMarker(
  const BoundingBox & bbox, const std_msgs::msg::Header & header, int marker_id) const
{
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;  // default valid quaternion if no rotation set
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

vision_msgs::msg::Detection3D BEVFusionNode::toDetection3D(
  const BoundingBox & bbox, const std_msgs::msg::Header & header) const
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
  // (check axis convention — nuScenes uses l=forward, w=lateral, h=vertical)

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

    if (detection_pub_) {
      detection_pub_->on_activate();
    }
    if (marker_pub_) {
      marker_pub_->on_activate();
    }

    if (diagnostic_updater_ && detection_pub_) {
      pub_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
        kOutputDetectionsTopic,
        *diagnostic_updater_,
        diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10),
        diagnostic_updater::TimeStampStatusParam());
      RCLCPP_INFO(this->get_logger(), "Topic diagnostics ready");
    }

    // Try to compute calibration matrices if camera info is already cached
    MultiCameraInfoMsg::ConstSharedPtr cached_camera_info;
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      cached_camera_info = cached_multi_camera_info_;
    }
    if (cached_camera_info) {
      computeCalibrationMatrices();
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "No cached multi camera info available at activation; calibration will be computed upon first message");
    }

    RCLCPP_INFO(this->get_logger(), "=============================================");
    RCLCPP_INFO(this->get_logger(), "Node activated successfully!");
    RCLCPP_INFO(this->get_logger(), "Subscribed to:");
    RCLCPP_INFO(this->get_logger(), "  - MultiCameraInfo: '%s'", multi_camera_info_sub_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "  - MultiImage: '%s'", multi_image_sub_->getTopic().c_str());
    RCLCPP_INFO(this->get_logger(), "  - LiDAR: '%s'", lidar_sub_->getTopic().c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to:");
    RCLCPP_INFO(this->get_logger(), "  - 3D Detections: '%s'", detection_pub_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "  - Markers: '%s'", marker_pub_->get_topic_name());
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

  if (detection_pub_) {
    detection_pub_->on_deactivate();
  }
  if (marker_pub_) {
    marker_pub_->on_deactivate();
  }

  sync_.reset();
  multi_image_sub_.reset();
  lidar_sub_.reset();
  multi_camera_info_sub_.reset();
  calibration_initialized_ = false;

  RCLCPP_INFO(this->get_logger(), "=============================================");
  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BEVFusionNode::on_cleanup(
  const rclcpp_lifecycle::State & /* prev_state */)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up BEVFusion node");

  sync_.reset();
  multi_image_sub_.reset();
  lidar_sub_.reset();
  multi_camera_info_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    cached_multi_camera_info_.reset();
  }
  calibration_initialized_ = false;

  RCLCPP_INFO(this->get_logger(), "Cleaning up publishers...");
  detection_pub_.reset();
  marker_pub_.reset();
  pub_diagnostic_.reset();
  diagnostic_updater_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
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
  RCLCPP_INFO(this->get_logger(), "  - MultiImage messages received: %lu", multi_image_msg_count_.load());
  RCLCPP_INFO(this->get_logger(), "  - LiDAR messages received: %lu", lidar_msg_count_.load());
  RCLCPP_INFO(this->get_logger(), "  - Synchronized callbacks: %lu", synced_msg_count_.load());

  RCLCPP_INFO(this->get_logger(), "Resetting all resources...");

  sync_.reset();
  multi_image_sub_.reset();
  lidar_sub_.reset();
  multi_camera_info_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    cached_multi_camera_info_.reset();
  }
  calibration_initialized_ = false;
  detection_pub_.reset();
  marker_pub_.reset();
  pub_diagnostic_.reset();
  diagnostic_updater_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  core_.reset();

  total_processed_ = 0;
  total_processing_time_ms_ = 0.0;
  last_processing_time_ms_ = 0.0;
  multi_image_msg_count_ = 0;
  lidar_msg_count_ = 0;
  synced_msg_count_ = 0;

  RCLCPP_INFO(this->get_logger(), "Node shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace wato::perception::bevfusion

RCLCPP_COMPONENTS_REGISTER_NODE(wato::perception::bevfusion::BEVFusionNode)
