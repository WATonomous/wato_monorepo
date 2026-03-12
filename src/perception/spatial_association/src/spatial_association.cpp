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

#include "spatial_association/spatial_association.hpp"

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

SpatialAssociationNode::SpatialAssociationNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("spatial_association_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Spatial Association lifecycle node created");
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpatialAssociationNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Spatial Association node");

  try {
    initializeParams();

    core_ = std::make_unique<SpatialAssociationCore>();

    working_colored_cluster_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    working_centroid_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    SpatialAssociationCore::ClusteringParams core_params;
    core_params.voxel_size = voxel_size_;
    core_params.euclid_cluster_tolerance = euclid_cluster_tolerance_;
    core_params.euclid_min_cluster_size = euclid_min_cluster_size_;
    core_params.euclid_max_cluster_size = euclid_max_cluster_size_;
    core_params.use_adaptive_clustering = use_adaptive_clustering_;
    core_params.euclid_close_threshold = euclid_close_threshold_;
    core_params.euclid_close_tolerance_mult = euclid_close_tolerance_mult_;
    core_params.merge_threshold = merge_threshold_;

    core_params.max_distance = this->get_parameter("quality_filter_params.max_distance").as_double();
    core_params.min_points = this->get_parameter("quality_filter_params.min_points").as_int();
    core_params.min_height = static_cast<float>(this->get_parameter("quality_filter_params.min_height").as_double());
    core_params.min_points_default = this->get_parameter("quality_filter_params.min_points_default").as_int();
    core_params.min_points_far = this->get_parameter("quality_filter_params.min_points_far").as_int();
    core_params.min_points_medium = this->get_parameter("quality_filter_params.min_points_medium").as_int();
    core_params.min_points_large = this->get_parameter("quality_filter_params.min_points_large").as_int();
    core_params.distance_threshold_far =
      this->get_parameter("quality_filter_params.distance_threshold_far").as_double();
    core_params.distance_threshold_medium =
      this->get_parameter("quality_filter_params.distance_threshold_medium").as_double();
    core_params.volume_threshold_large =
      static_cast<float>(this->get_parameter("quality_filter_params.volume_threshold_large").as_double());
    core_params.min_density = static_cast<float>(this->get_parameter("quality_filter_params.min_density").as_double());
    core_params.max_density = static_cast<float>(this->get_parameter("quality_filter_params.max_density").as_double());
    core_params.max_dimension =
      static_cast<float>(this->get_parameter("quality_filter_params.max_dimension").as_double());
    core_params.max_aspect_ratio =
      static_cast<float>(this->get_parameter("quality_filter_params.max_aspect_ratio").as_double());

    core_->setParams(core_params);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Node configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Configuration failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpatialAssociationNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Spatial Association node");

  try {
    // Subscribers
    non_ground_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      kNonGroundCloud, 10, std::bind(&SpatialAssociationNode::nonGroundCloudCallback, this, std::placeholders::_1));

    multi_camera_info_sub_ = this->create_subscription<deep_msgs::msg::MultiCameraInfo>(
      kMultiCameraInfo,
      rclcpp::SensorDataQoS(),
      std::bind(&SpatialAssociationNode::multiCameraInfoCallback, this, std::placeholders::_1));

    if (publish_visualization_) {
      multi_image_sub_ = this->create_subscription<deep_msgs::msg::MultiImageCompressed>(
        kMultiImage,
        rclcpp::SensorDataQoS(),
        std::bind(&SpatialAssociationNode::multiImageCallback, this, std::placeholders::_1));
    }

    dets_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      kDetections, 10, std::bind(&SpatialAssociationNode::detectionCallback, this, std::placeholders::_1));

    // Publishers
    detection_3d_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(kDetection3d, 10);
    if (detection_3d_pub_) {
      detection_3d_pub_->on_activate();
    }

    if (publish_bounding_box_) {
      bounding_box_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kBoundingBox, 10);
      if (bounding_box_pub_) {
        bounding_box_pub_->on_activate();
      }
      RCLCPP_INFO(this->get_logger(), "Bounding box publisher enabled");
    }

    if (publish_visualization_) {
      filtered_lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(kFilteredLidar, 10);
      cluster_centroid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(kClusterCentroid, 10);
      if (filtered_lidar_pub_) {
        filtered_lidar_pub_->on_activate();
      }
      if (cluster_centroid_pub_) {
        cluster_centroid_pub_->on_activate();
      }
      RCLCPP_INFO(this->get_logger(), "Debug visualization publishers enabled");
    }

    RCLCPP_INFO(this->get_logger(), "Node activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Activation failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpatialAssociationNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Spatial Association node");

  if (detection_3d_pub_) {
    detection_3d_pub_->on_deactivate();
  }
  if (bounding_box_pub_) {
    bounding_box_pub_->on_deactivate();
  }
  if (filtered_lidar_pub_) {
    filtered_lidar_pub_->on_deactivate();
  }
  if (cluster_centroid_pub_) {
    cluster_centroid_pub_->on_deactivate();
  }

  non_ground_cloud_sub_.reset();
  multi_image_sub_.reset();
  multi_camera_info_sub_.reset();
  dets_sub_.reset();
  latest_multi_image_.reset();

  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpatialAssociationNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Spatial Association node");

  non_ground_cloud_sub_.reset();
  multi_image_sub_.reset();
  multi_camera_info_sub_.reset();
  dets_sub_.reset();
  detection_3d_pub_.reset();
  bounding_box_pub_.reset();
  filtered_lidar_pub_.reset();
  cluster_centroid_pub_.reset();
  core_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  latest_multi_camera_info_.reset();
  latest_multi_image_.reset();
  frame_id_to_index_.clear();
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    filtered_point_cloud_.reset();
    cluster_indices.clear();
  }
  working_colored_cluster_.reset();
  working_centroid_cloud_.reset();

  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpatialAssociationNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    this->get_logger(), "Shutting down Spatial Association node from state: %s", previous_state.label().c_str());

  non_ground_cloud_sub_.reset();
  multi_image_sub_.reset();
  multi_camera_info_sub_.reset();
  dets_sub_.reset();
  detection_3d_pub_.reset();
  bounding_box_pub_.reset();
  filtered_lidar_pub_.reset();
  cluster_centroid_pub_.reset();
  core_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  latest_multi_camera_info_.reset();
  latest_multi_image_.reset();
  frame_id_to_index_.clear();
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    filtered_point_cloud_.reset();
    cluster_indices.clear();
  }
  working_colored_cluster_.reset();
  working_centroid_cloud_.reset();

  RCLCPP_INFO(this->get_logger(), "Node shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Parameter initialisation
// ---------------------------------------------------------------------------

void SpatialAssociationNode::initializeParams()
{
  this->declare_parameter<std::string>("lidar_frame", "lidar_cc");

  this->declare_parameter<bool>("publish_bounding_box", true);
  this->declare_parameter<bool>("publish_visualization", false);
  this->declare_parameter<bool>("debug_logging", false);

  this->declare_parameter<float>("voxel_size", 0.2f);

  this->declare_parameter<double>("euclid_params.cluster_tolerance", 0.5);
  this->declare_parameter<int>("euclid_params.min_cluster_size", 50);
  this->declare_parameter<int>("euclid_params.max_cluster_size", 700);
  this->declare_parameter<bool>("euclid_params.use_adaptive_clustering", true);
  this->declare_parameter<double>("euclid_params.close_threshold", 10.0);
  this->declare_parameter<double>("euclid_params.close_tolerance_mult", 1.5);

  this->declare_parameter<double>("merge_threshold", 0.3);

  this->declare_parameter<float>("object_detection_confidence", 0.4);

  this->declare_parameter<double>("quality_filter_params.max_distance", 60.0);
  this->declare_parameter<int>("quality_filter_params.min_points", 5);
  this->declare_parameter<double>("quality_filter_params.min_height", 0.15);
  this->declare_parameter<int>("quality_filter_params.min_points_default", 10);
  this->declare_parameter<int>("quality_filter_params.min_points_far", 8);
  this->declare_parameter<int>("quality_filter_params.min_points_medium", 12);
  this->declare_parameter<int>("quality_filter_params.min_points_large", 30);
  this->declare_parameter<double>("quality_filter_params.distance_threshold_far", 30.0);
  this->declare_parameter<double>("quality_filter_params.distance_threshold_medium", 20.0);
  this->declare_parameter<double>("quality_filter_params.volume_threshold_large", 8.0);
  this->declare_parameter<double>("quality_filter_params.min_density", 5.0);
  this->declare_parameter<double>("quality_filter_params.max_density", 1000.0);
  this->declare_parameter<double>("quality_filter_params.max_dimension", 15.0);
  this->declare_parameter<double>("quality_filter_params.max_aspect_ratio", 15.0);

  // Projection/utils params (orientation, merge, IoU, viz)
  const std::string pu("projection_utils_params.");
  this->declare_parameter<double>(pu + "marker_lifetime_s", 0.5);
  this->declare_parameter<double>(pu + "marker_alpha", 0.2);
  this->declare_parameter<double>(pu + "min_iou_threshold", 0.15);
  this->declare_parameter<double>(pu + "ar_front_view_threshold", 1.2);
  this->declare_parameter<int>(pu + "outlier_rejection_point_count", 30);
  this->declare_parameter<double>(pu + "outlier_sigma_multiplier", 4.5);
  this->declare_parameter<double>(pu + "xy_extent_percentile_low", 5.0);
  this->declare_parameter<double>(pu + "xy_extent_percentile_high", 95.0);
  this->declare_parameter<double>(pu + "z_extent_percentile_low", 2.0);
  this->declare_parameter<double>(pu + "z_extent_percentile_high", 98.0);
  this->declare_parameter<int>(pu + "min_points_for_fit", 3);
  this->declare_parameter<int>(pu + "default_sample_point_count", 64);
  this->declare_parameter<double>(pu + "orientation_search_step_degrees", 2.0);
  this->declare_parameter<double>(pu + "min_camera_z_distance", 1.0);
  this->declare_parameter<double>(pu + "detection_score_weight", 0.6);
  this->declare_parameter<double>(pu + "iou_score_weight", 0.4);

  publish_bounding_box_ = this->get_parameter("publish_bounding_box").as_bool();
  publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
  voxel_size_ = static_cast<float>(this->get_parameter("voxel_size").as_double());
  lidar_frame_ = this->get_parameter("lidar_frame").as_string();

  const std::string pu_get("projection_utils_params.");

  euclid_cluster_tolerance_ = this->get_parameter("euclid_params.cluster_tolerance").as_double();
  euclid_min_cluster_size_ = this->get_parameter("euclid_params.min_cluster_size").as_int();
  euclid_max_cluster_size_ = this->get_parameter("euclid_params.max_cluster_size").as_int();
  use_adaptive_clustering_ = this->get_parameter("euclid_params.use_adaptive_clustering").as_bool();
  euclid_close_threshold_ = this->get_parameter("euclid_params.close_threshold").as_double();
  euclid_close_tolerance_mult_ = this->get_parameter("euclid_params.close_tolerance_mult").as_double();

  merge_threshold_ = this->get_parameter("merge_threshold").as_double();
  object_detection_confidence_ = this->get_parameter("object_detection_confidence").as_double();

  debug_logging_ = this->get_parameter("debug_logging").as_bool();

  // Projection/utils: apply to static params used by projection_utils.cpp
  ProjectionUtils::ProjectionUtilsParams proj_params;
  proj_params.marker_lifetime_s = this->get_parameter(pu_get + "marker_lifetime_s").as_double();
  proj_params.marker_alpha = static_cast<float>(this->get_parameter(pu_get + "marker_alpha").as_double());
  proj_params.min_iou_threshold = this->get_parameter(pu_get + "min_iou_threshold").as_double();
  proj_params.ar_front_view_threshold = this->get_parameter(pu_get + "ar_front_view_threshold").as_double();
  proj_params.outlier_rejection_point_count = static_cast<size_t>(this->get_parameter(pu_get + "outlier_rejection_point_count").as_int());
  proj_params.outlier_sigma_multiplier = this->get_parameter(pu_get + "outlier_sigma_multiplier").as_double();
  proj_params.xy_extent_percentile_low = this->get_parameter(pu_get + "xy_extent_percentile_low").as_double();
  proj_params.xy_extent_percentile_high = this->get_parameter(pu_get + "xy_extent_percentile_high").as_double();
  proj_params.z_extent_percentile_low = this->get_parameter(pu_get + "z_extent_percentile_low").as_double();
  proj_params.z_extent_percentile_high = this->get_parameter(pu_get + "z_extent_percentile_high").as_double();
  proj_params.min_points_for_fit = static_cast<size_t>(this->get_parameter(pu_get + "min_points_for_fit").as_int());
  proj_params.default_sample_point_count = static_cast<size_t>(this->get_parameter(pu_get + "default_sample_point_count").as_int());
  proj_params.orientation_search_step_degrees = this->get_parameter(pu_get + "orientation_search_step_degrees").as_double();
  proj_params.min_camera_z_distance = this->get_parameter(pu_get + "min_camera_z_distance").as_double();
  proj_params.detection_score_weight = this->get_parameter(pu_get + "detection_score_weight").as_double();
  proj_params.iou_score_weight = this->get_parameter(pu_get + "iou_score_weight").as_double();
  ProjectionUtils::setParams(proj_params);

  if (debug_logging_) {
    RCLCPP_INFO(this->get_logger(), "Debug logging is ENABLED for spatial_association");
  }
  RCLCPP_INFO(this->get_logger(), "Parameters initialized");
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void SpatialAssociationNode::multiCameraInfoCallback(const deep_msgs::msg::MultiCameraInfo::SharedPtr msg)
{
  // Keep one SharedPtr to the message; index by frame_id. No copy of CameraInfo or image data.
  latest_multi_camera_info_ = msg;
  frame_id_to_index_.clear();
  size_t skipped = 0;
  for (size_t i = 0; i < msg->camera_infos.size(); ++i) {
    const std::string & frame_id = msg->camera_infos[i].header.frame_id;
    if (frame_id.empty()) {
      skipped++;
      continue;
    }
    frame_id_to_index_[frame_id] = i;
  }
  if (skipped > 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *get_clock(),
      5000,
      "MultiCameraInfo: skipped %zu entry/entries with empty frame_id",
      skipped);
  }
  if (debug_logging_ && !msg->camera_infos.empty()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Updated CameraInfo index from MultiCameraInfo: %zu camera(s) (frame_ids: keys for TF and detections)",
      frame_id_to_index_.size());
  }
}

void SpatialAssociationNode::multiImageCallback(const deep_msgs::msg::MultiImageCompressed::SharedPtr msg)
{
  // Store SharedPtr only; used for visualization when publish_visualization is true. No copy.
  latest_multi_image_ = msg;
}

void SpatialAssociationNode::nonGroundCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (debug_logging_) {
    RCLCPP_INFO(this->get_logger(), "Received non-ground cloud with %d points", msg->width * msg->height);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *input_cloud);

  if (input_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty non-ground cloud");
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_lidar_header_ = msg->header;
    filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cluster_indices.clear();
    return;
  }

  // Run voxel + clustering off the callback thread so detection callbacks can run at full rate.
  const std_msgs::msg::Header header = msg->header;
  std::thread([this, input_cloud, header]() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    core_->processPointCloud(input_cloud, filtered);
    std::vector<pcl::PointIndices> indices;
    core_->performClustering(filtered, indices);
    const size_t num_points = filtered->size();
    const size_t num_clusters = indices.size();
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      latest_lidar_header_ = header;
      filtered_point_cloud_ = std::move(filtered);
      cluster_indices = std::move(indices);
    }
    if (debug_logging_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Processed non-ground cloud: %zu points, %zu clusters",
        num_points,
        num_clusters);
    }
  }).detach();
}

void SpatialAssociationNode::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  // frame_id must match MultiCameraInfo.camera_infos[].header.frame_id and the camera
  // optical frame name in TF (sensor_interfacing). deep_ros uses the same camera name for both.
  const std::string & frame_id = msg->header.frame_id;

  if (!latest_multi_camera_info_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No MultiCameraInfo received yet");
    return;
  }
  const auto it = frame_id_to_index_.find(frame_id);
  if (it == frame_id_to_index_.end()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "No CameraInfo for '%s', skipping detections", frame_id.c_str());
    return;
  }
  const std::array<double, 12> & projection_matrix = latest_multi_camera_info_->camera_infos[it->second].p;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<pcl::PointIndices> indices;
  std_msgs::msg::Header lidar_header;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = filtered_point_cloud_;
    indices = cluster_indices;
    lidar_header = latest_lidar_header_;
  }

  if (!cloud || cloud->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No non-ground cloud data, skipping detection processing");
    return;
  }

  if (indices.empty()) {
    RCLCPP_DEBUG(get_logger(), "No clusters available");
    return;
  }

  // Look up lidar -> camera transform from TF (published by sensor_interfacing).
  geometry_msgs::msg::TransformStamped tf_lidar_to_cam;
  try {
    tf_lidar_to_cam = tf_buffer_->lookupTransform(frame_id, lidar_frame_, tf2::TimePointZero);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "TF %s->%s failed: %s", lidar_frame_.c_str(), frame_id.c_str(), e.what());
    return;
  }

  auto detection_results = processDetections(*msg, tf_lidar_to_cam, projection_matrix, cloud, indices, lidar_header);

  // Publish 3D detections
  if (detection_3d_pub_ && detection_3d_pub_->is_activated()) {
    detection_results.detections3d.header = lidar_header;
    detection_results.detections3d.header.stamp = this->get_clock()->now();
    detection_3d_pub_->publish(detection_results.detections3d);
  }

  // Publish bounding box markers
  if (publish_bounding_box_ && bounding_box_pub_ && bounding_box_pub_->is_activated()) {
    for (auto & marker : detection_results.bboxes.markers) {
      marker.ns = frame_id;
    }
    bounding_box_pub_->publish(detection_results.bboxes);
  }

  // Publish debug visualisation topics
  if (publish_visualization_) {
    if (filtered_lidar_pub_ && filtered_lidar_pub_->is_activated() && detection_results.colored_cluster) {
      sensor_msgs::msg::PointCloud2 pcl2_msg;
      pcl::toROSMsg(*detection_results.colored_cluster, pcl2_msg);
      pcl2_msg.header = lidar_header;
      filtered_lidar_pub_->publish(pcl2_msg);
    }
    if (cluster_centroid_pub_ && cluster_centroid_pub_->is_activated() && detection_results.centroid_cloud) {
      sensor_msgs::msg::PointCloud2 pcl2_msg;
      pcl::toROSMsg(*detection_results.centroid_cloud, pcl2_msg);
      pcl2_msg.header = lidar_header;
      cluster_centroid_pub_->publish(pcl2_msg);
    }
  }
}

// ---------------------------------------------------------------------------
// Core processing
// ---------------------------------------------------------------------------

DetectionOutputs SpatialAssociationNode::processDetections(
  const vision_msgs::msg::Detection2DArray & detection,
  const geometry_msgs::msg::TransformStamped & transform,
  const std::array<double, 12> & projection_matrix,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::vector<pcl::PointIndices> & indices,
  const std_msgs::msg::Header & lidar_header)
{
  DetectionOutputs detection_outputs;

  if (!cloud || cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Non-ground cloud data not available or empty");
    return detection_outputs;
  }

  if (indices.empty()) {
    return detection_outputs;
  }

  std::vector<ProjectionUtils::ClusterCandidate> candidates =
    ProjectionUtils::buildCandidates(cloud, indices);

  const auto & q = core_->getParams();
  ProjectionUtils::filterCandidatesByPhysicsConstraints(
    candidates,
    q.max_distance,
    q.min_points,
    q.min_height,
    q.min_points_default,
    q.min_points_far,
    q.min_points_medium,
    q.min_points_large,
    q.distance_threshold_far,
    q.distance_threshold_medium,
    q.volume_threshold_large,
    q.min_density,
    q.max_density,
    q.max_dimension,
    q.max_aspect_ratio);

  if (debug_logging_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Camera %s: %zu clusters before IOU filtering, %zu detections",
      transform.header.frame_id.c_str(),
      candidates.size(),
      detection.detections.size());
  }

  ProjectionUtils::assignCandidatesToDetectionsByIOU(
    candidates, detection, transform, projection_matrix, object_detection_confidence_);

  if (debug_logging_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Camera %s: %zu clusters kept after IOU filtering",
      transform.header.frame_id.c_str(),
      candidates.size());
  }

  ProjectionUtils::filterCandidatesByClassAwareConstraints(
    candidates,
    detection,
    q.max_distance,
    q.min_points,
    q.min_height,
    q.min_points_default,
    q.min_points_far,
    q.min_points_medium,
    q.min_points_large,
    q.distance_threshold_far,
    q.distance_threshold_medium,
    q.volume_threshold_large,
    q.min_density,
    q.max_density,
    q.max_dimension,
    q.max_aspect_ratio);

  std::vector<pcl::PointIndices> out_indices = ProjectionUtils::extractIndices(candidates);
  auto boxes = core_->computeClusterBoxes(cloud, out_indices);

  if (publish_bounding_box_) {
    detection_outputs.bboxes = ProjectionUtils::computeBoundingBox(boxes, out_indices, lidar_header);
  }

  detection_outputs.detections3d =
    ProjectionUtils::compute3DDetection(boxes, candidates, lidar_header, detection);

  if (publish_visualization_) {
    detection_outputs.colored_cluster = working_colored_cluster_;
    core_->assignClusterColors(cloud, out_indices, detection_outputs.colored_cluster);

    detection_outputs.centroid_cloud = working_centroid_cloud_;
    core_->computeClusterCentroids(cloud, out_indices, detection_outputs.centroid_cloud);
  }
  return detection_outputs;
}

RCLCPP_COMPONENTS_REGISTER_NODE(SpatialAssociationNode)
