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

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

SpatialAssociationNode::SpatialAssociationNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("spatial_association_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Spatial Association lifecycle node created");
}

SpatialAssociationNode::~SpatialAssociationNode()
{
  stopWorker();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpatialAssociationNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Spatial Association node");

  try {
    initializeParams();

    core_ = std::make_unique<SpatialAssociationCore>();

    SpatialAssociationCore::ClusteringParams core_params;
    core_params.voxel_size = voxel_size_;
    core_params.euclid_cluster_tolerance = euclid_cluster_tolerance_;
    core_params.euclid_min_cluster_size = euclid_min_cluster_size_;
    core_params.euclid_max_cluster_size = euclid_max_cluster_size_;
    core_params.use_adaptive_clustering = use_adaptive_clustering_;
    core_params.euclid_close_threshold = euclid_close_threshold_;
    core_params.euclid_close_tolerance_mult = euclid_close_tolerance_mult_;
    core_params.merge_threshold = merge_threshold_;

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
    non_ground_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      kNonGroundCloud,
      rclcpp::SensorDataQoS(),
      std::bind(&SpatialAssociationNode::nonGroundCloudCallback, this, std::placeholders::_1));

    multi_camera_info_sub_ = this->create_subscription<deep_msgs::msg::MultiCameraInfo>(
      kMultiCameraInfo,
      rclcpp::SensorDataQoS(),
      std::bind(&SpatialAssociationNode::multiCameraInfoCallback, this, std::placeholders::_1));

    if (publish_image_visualization_) {
      multi_image_sub_ = this->create_subscription<deep_msgs::msg::MultiImageCompressed>(
        kMultiImage,
        rclcpp::SensorDataQoS(),
        std::bind(&SpatialAssociationNode::multiImageCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Image visualization enabled (multi_image subscription)");
    }

    {
      auto det_qos = rclcpp::QoS(rclcpp::KeepLast(10));
      det_qos.best_effort();
      dets_sub_ = this->create_subscription<deep_msgs::msg::MultiDetection2DArray>(
        kDetections, det_qos, std::bind(&SpatialAssociationNode::multiDetectionCallback, this, std::placeholders::_1));
    }

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

    worker_shutdown_ = false;
    worker_thread_ = std::thread(&SpatialAssociationNode::workerLoop, this);

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

  non_ground_cloud_sub_.reset();
  multi_image_sub_.reset();
  multi_camera_info_sub_.reset();
  dets_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_multi_image_.reset();
  }

  stopWorker();

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
  stopWorker();
  detection_3d_pub_.reset();
  bounding_box_pub_.reset();
  filtered_lidar_pub_.reset();
  cluster_centroid_pub_.reset();
  core_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    latest_multi_camera_info_.reset();
    frame_id_to_index_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_multi_image_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    filtered_point_cloud_.reset();
    cluster_indices.clear();
  }
  {
    std::lock_guard<std::mutex> lock(candidates_cache_mutex_);
    cached_candidates_.clear();
    cached_candidates_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

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
  stopWorker();
  detection_3d_pub_.reset();
  bounding_box_pub_.reset();
  filtered_lidar_pub_.reset();
  cluster_centroid_pub_.reset();
  core_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    latest_multi_camera_info_.reset();
    frame_id_to_index_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_multi_image_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    filtered_point_cloud_.reset();
    cluster_indices.clear();
  }
  {
    std::lock_guard<std::mutex> lock(candidates_cache_mutex_);
    cached_candidates_.clear();
    cached_candidates_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  RCLCPP_INFO(this->get_logger(), "Node shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

namespace
{
template <typename T>
void declareIfMissing(rclcpp_lifecycle::LifecycleNode * node, const std::string & name, const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
}
}  // namespace

void SpatialAssociationNode::initializeParams()
{
  declareIfMissing(this, "lidar_frame", std::string("lidar_cc"));
  declareIfMissing(this, "max_detection_cloud_stamp_delta", 0.0);

  declareIfMissing(this, "publish_bounding_box", true);
  declareIfMissing(this, "publish_visualization", false);
  declareIfMissing(this, "publish_image_visualization", false);
  declareIfMissing(this, "debug_logging", false);

  declareIfMissing(this, "voxel_size", 0.2f);

  declareIfMissing(this, "euclid_params.cluster_tolerance", 0.5);
  declareIfMissing(this, "euclid_params.min_cluster_size", 50);
  declareIfMissing(this, "euclid_params.max_cluster_size", 700);
  declareIfMissing(this, "euclid_params.use_adaptive_clustering", true);
  declareIfMissing(this, "euclid_params.close_threshold", 10.0);
  declareIfMissing(this, "euclid_params.close_tolerance_mult", 1.5);

  declareIfMissing(this, "merge_threshold", 0.3);

  declareIfMissing(this, "object_detection_confidence", 0.4f);

  declareIfMissing(this, "quality_filter_params.max_distance", 60.0);
  declareIfMissing(this, "quality_filter_params.min_points", 5);
  declareIfMissing(this, "quality_filter_params.min_height", 0.15);
  declareIfMissing(this, "quality_filter_params.min_points_default", 10);
  declareIfMissing(this, "quality_filter_params.min_points_far", 8);
  declareIfMissing(this, "quality_filter_params.min_points_medium", 12);
  declareIfMissing(this, "quality_filter_params.min_points_large", 30);
  declareIfMissing(this, "quality_filter_params.distance_threshold_far", 30.0);
  declareIfMissing(this, "quality_filter_params.distance_threshold_medium", 20.0);
  declareIfMissing(this, "quality_filter_params.volume_threshold_large", 8.0);
  declareIfMissing(this, "quality_filter_params.min_density", 5.0);
  declareIfMissing(this, "quality_filter_params.max_density", 1000.0);
  declareIfMissing(this, "quality_filter_params.max_dimension", 15.0);
  declareIfMissing(this, "quality_filter_params.max_aspect_ratio", 15.0);

  const std::string pu("projection_utils_params.");
  declareIfMissing(this, pu + "marker_lifetime_s", 0.5);
  declareIfMissing(this, pu + "marker_alpha", 0.2);
  declareIfMissing(this, pu + "min_iou_threshold", 0.15);
  declareIfMissing(this, pu + "ar_front_view_threshold", 1.2);
  declareIfMissing(this, pu + "outlier_rejection_point_count", 30);
  declareIfMissing(this, pu + "outlier_sigma_multiplier", 4.5);
  declareIfMissing(this, pu + "xy_extent_percentile_low", 5.0);
  declareIfMissing(this, pu + "xy_extent_percentile_high", 95.0);
  declareIfMissing(this, pu + "z_extent_percentile_low", 2.0);
  declareIfMissing(this, pu + "z_extent_percentile_high", 98.0);
  declareIfMissing(this, pu + "min_points_for_fit", 3);
  declareIfMissing(this, pu + "default_sample_point_count", 64);
  declareIfMissing(this, pu + "orientation_search_step_degrees", 2.0);
  declareIfMissing(this, pu + "min_camera_z_distance", 1.0);
  declareIfMissing(this, pu + "detection_score_weight", 0.6);
  declareIfMissing(this, pu + "iou_score_weight", 0.4);
  declareIfMissing(this, pu + "image_width", 1280);
  declareIfMissing(this, pu + "image_height", 1024);
  declareIfMissing(this, pu + "enable_second_pass_fallback", false);
  declareIfMissing(this, pu + "second_pass_min_iou", 0.05);
  declareIfMissing(this, pu + "max_unassigned_detections_second_pass", 10);

  publish_bounding_box_ = this->get_parameter("publish_bounding_box").as_bool();
  publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
  publish_image_visualization_ = this->get_parameter("publish_image_visualization").as_bool();
  voxel_size_ = static_cast<float>(this->get_parameter("voxel_size").as_double());
  lidar_frame_ = this->get_parameter("lidar_frame").as_string();
  max_detection_cloud_stamp_delta_ = this->get_parameter("max_detection_cloud_stamp_delta").as_double();

  euclid_cluster_tolerance_ = this->get_parameter("euclid_params.cluster_tolerance").as_double();
  euclid_min_cluster_size_ = this->get_parameter("euclid_params.min_cluster_size").as_int();
  euclid_max_cluster_size_ = this->get_parameter("euclid_params.max_cluster_size").as_int();
  use_adaptive_clustering_ = this->get_parameter("euclid_params.use_adaptive_clustering").as_bool();
  euclid_close_threshold_ = this->get_parameter("euclid_params.close_threshold").as_double();
  euclid_close_tolerance_mult_ = this->get_parameter("euclid_params.close_tolerance_mult").as_double();

  merge_threshold_ = this->get_parameter("merge_threshold").as_double();
  object_detection_confidence_ = this->get_parameter("object_detection_confidence").as_double();

  debug_logging_ = this->get_parameter("debug_logging").as_bool();

  projection_utils::ProjectionUtilsParams proj_params;
  proj_params.marker_lifetime_s = this->get_parameter(pu + "marker_lifetime_s").as_double();
  proj_params.marker_alpha = static_cast<float>(this->get_parameter(pu + "marker_alpha").as_double());
  proj_params.min_iou_threshold = this->get_parameter(pu + "min_iou_threshold").as_double();
  proj_params.ar_front_view_threshold = this->get_parameter(pu + "ar_front_view_threshold").as_double();
  proj_params.outlier_rejection_point_count =
    static_cast<size_t>(this->get_parameter(pu + "outlier_rejection_point_count").as_int());
  proj_params.outlier_sigma_multiplier = this->get_parameter(pu + "outlier_sigma_multiplier").as_double();
  proj_params.xy_extent_percentile_low = this->get_parameter(pu + "xy_extent_percentile_low").as_double();
  proj_params.xy_extent_percentile_high = this->get_parameter(pu + "xy_extent_percentile_high").as_double();
  proj_params.z_extent_percentile_low = this->get_parameter(pu + "z_extent_percentile_low").as_double();
  proj_params.z_extent_percentile_high = this->get_parameter(pu + "z_extent_percentile_high").as_double();
  proj_params.min_points_for_fit = static_cast<size_t>(this->get_parameter(pu + "min_points_for_fit").as_int());
  proj_params.default_sample_point_count =
    static_cast<size_t>(this->get_parameter(pu + "default_sample_point_count").as_int());
  proj_params.orientation_search_step_degrees = this->get_parameter(pu + "orientation_search_step_degrees").as_double();
  proj_params.min_camera_z_distance = this->get_parameter(pu + "min_camera_z_distance").as_double();
  proj_params.detection_score_weight = this->get_parameter(pu + "detection_score_weight").as_double();
  proj_params.iou_score_weight = this->get_parameter(pu + "iou_score_weight").as_double();
  proj_params.image_width = this->get_parameter(pu + "image_width").as_int();
  proj_params.image_height = this->get_parameter(pu + "image_height").as_int();
  proj_params.enable_second_pass_fallback = this->get_parameter(pu + "enable_second_pass_fallback").as_bool();
  proj_params.second_pass_min_iou = this->get_parameter(pu + "second_pass_min_iou").as_double();
  proj_params.max_unassigned_detections_second_pass =
    this->get_parameter(pu + "max_unassigned_detections_second_pass").as_int();

  proj_params.quality_max_distance = this->get_parameter("quality_filter_params.max_distance").as_double();
  proj_params.quality_min_points = this->get_parameter("quality_filter_params.min_points").as_int();
  proj_params.quality_min_height =
    static_cast<float>(this->get_parameter("quality_filter_params.min_height").as_double());
  proj_params.quality_min_points_default = this->get_parameter("quality_filter_params.min_points_default").as_int();
  proj_params.quality_min_points_far = this->get_parameter("quality_filter_params.min_points_far").as_int();
  proj_params.quality_min_points_medium = this->get_parameter("quality_filter_params.min_points_medium").as_int();
  proj_params.quality_min_points_large = this->get_parameter("quality_filter_params.min_points_large").as_int();
  proj_params.quality_distance_threshold_far =
    this->get_parameter("quality_filter_params.distance_threshold_far").as_double();
  proj_params.quality_distance_threshold_medium =
    this->get_parameter("quality_filter_params.distance_threshold_medium").as_double();
  proj_params.quality_volume_threshold_large =
    static_cast<float>(this->get_parameter("quality_filter_params.volume_threshold_large").as_double());
  proj_params.quality_min_density =
    static_cast<float>(this->get_parameter("quality_filter_params.min_density").as_double());
  proj_params.quality_max_density =
    static_cast<float>(this->get_parameter("quality_filter_params.max_density").as_double());
  proj_params.quality_max_dimension =
    static_cast<float>(this->get_parameter("quality_filter_params.max_dimension").as_double());
  proj_params.quality_max_aspect_ratio =
    static_cast<float>(this->get_parameter("quality_filter_params.max_aspect_ratio").as_double());

  projection_utils::setParams(proj_params);

  if (debug_logging_) {
    RCLCPP_INFO(this->get_logger(), "Debug logging is ENABLED for spatial_association");
  }
  RCLCPP_INFO(this->get_logger(), "Parameters initialized");
}

void SpatialAssociationNode::multiCameraInfoCallback(const deep_msgs::msg::MultiCameraInfo::SharedPtr msg)
{
  size_t num_entries = 0;
  size_t skipped = 0;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    latest_multi_camera_info_ = msg;
    frame_id_to_index_.clear();
    for (size_t i = 0; i < msg->camera_infos.size(); ++i) {
      const std::string & frame_id = msg->camera_infos[i].header.frame_id;
      if (frame_id.empty()) {
        skipped++;
        continue;
      }
      frame_id_to_index_[frame_id] = i;
    }
    num_entries = frame_id_to_index_.size();
  }
  {
    std::lock_guard<std::mutex> lock(transform_cache_mutex_);
    lidar_to_cam_transform_cache_.clear();
  }
  if (skipped > 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *get_clock(),
      5000,
      "MultiCameraInfo: skipped %zu entry/entries with empty frame_id",
      skipped);
  }
  if (debug_logging_ && num_entries > 0) {
    RCLCPP_INFO(
      this->get_logger(),
      "Updated CameraInfo index from MultiCameraInfo: %zu camera(s) (frame_ids: keys for TF and detections)",
      num_entries);
  }
}

void SpatialAssociationNode::multiImageCallback(const deep_msgs::msg::MultiImageCompressed::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  latest_multi_image_ = msg;
}

void SpatialAssociationNode::stopWorker()
{
  worker_shutdown_ = true;
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    pending_cloud_.reset();
    pending_header_ = std_msgs::msg::Header{};
    pending_job_id_ = 0;
    job_cv_.notify_all();
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void SpatialAssociationNode::workerLoop()
{
  while (true) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr my_cloud;
    std_msgs::msg::Header my_header;
    {
      std::unique_lock<std::mutex> lock(job_mutex_);
      job_cv_.wait(lock, [this] { return worker_shutdown_ || pending_job_id_ != 0; });
      if (worker_shutdown_) {
        break;
      }
      my_cloud = std::move(pending_cloud_);
      my_header = pending_header_;
      pending_job_id_ = 0;
    }
    if (!my_cloud || my_cloud->empty()) {
      continue;
    }
    if (!core_) {
      continue;
    }
    if (worker_shutdown_) {
      break;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    core_->processPointCloud(my_cloud, filtered);
    if (worker_shutdown_) {
      break;
    }
    std::vector<pcl::PointIndices> indices;
    core_->performClustering(filtered, indices);
    const size_t num_points = filtered->size();
    const size_t num_clusters = indices.size();

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      latest_lidar_header_ = my_header;
      filtered_point_cloud_ = std::move(filtered);
      cluster_indices = std::move(indices);
    }
    {
      std::lock_guard<std::mutex> lock(candidates_cache_mutex_);
      cached_candidates_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    if (debug_logging_) {
      RCLCPP_INFO(this->get_logger(), "Processed non-ground cloud: %zu points, %zu clusters", num_points, num_clusters);
    }
  }
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
    ++cloud_job_id_;  // invalidate any in-flight worker job so it won't overwrite empty state
    latest_lidar_header_ = msg->header;
    filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cluster_indices.clear();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    pending_cloud_ = std::move(input_cloud);
    pending_header_ = msg->header;
    pending_job_id_ = ++cloud_job_id_;
    job_cv_.notify_one();
  }
}

void SpatialAssociationNode::multiDetectionCallback(const deep_msgs::msg::MultiDetection2DArray::SharedPtr msg)
{
  // MultiDetection2DArray: header + camera_detections[] (each Detection2DArray has header.frame_id and detections[])
  if (msg->camera_detections.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "MultiDetection2DArray has empty camera_detections — no per-camera 2D detections to associate");
    return;
  }

  size_t total_2d = 0;
  for (const auto & cam : msg->camera_detections) {
    total_2d += cam.detections.size();
  }
  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "MultiDetection2DArray: %zu camera(s), %zu total 2D detections",
    msg->camera_detections.size(),
    total_2d);

  deep_msgs::msg::MultiCameraInfo::SharedPtr info;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    info = latest_multi_camera_info_;
  }
  if (!info) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No MultiCameraInfo received yet");
    return;
  }

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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No clusters available from non-ground cloud");
    return;
  }

  std::vector<projection_utils::ClusterCandidate> cached_candidates;
  {
    std::lock_guard<std::mutex> lock(candidates_cache_mutex_);
    const rclcpp::Time cloud_stamp(lidar_header.stamp, get_clock()->get_clock_type());
    if (cloud_stamp != cached_candidates_stamp_ || cached_candidates_.empty()) {
      cached_candidates_ = projection_utils::buildCandidates(cloud, indices);
      cached_candidates_stamp_ = cloud_stamp;
    }
    cached_candidates = cached_candidates_;
  }

  if (cached_candidates.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No cluster candidates available");
    return;
  }

  const rclcpp::Time cloud_time(lidar_header.stamp, get_clock()->get_clock_type());

  vision_msgs::msg::Detection3DArray merged_detections3d;
  merged_detections3d.header = lidar_header;
  visualization_msgs::msg::MarkerArray merged_bboxes;

  for (const auto & camera_detections : msg->camera_detections) {
    const std::string & frame_id = camera_detections.header.frame_id;

    std::optional<size_t> camera_index;
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      const auto it = frame_id_to_index_.find(frame_id);
      if (it != frame_id_to_index_.end()) {
        camera_index = it->second;
      }
    }
    if (!camera_index.has_value()) {
      std::string known;
      {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        for (const auto & kv : frame_id_to_index_) {
          if (!known.empty()) known += ", ";
          known += kv.first;
        }
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No CameraInfo matching frame_id '%s' — detection skipped. Known frame_ids: [%s]. "
        "Ensure 2D detection headers match MultiCameraInfo (same camera order and frame_id).",
        frame_id.c_str(),
        known.c_str());
      continue;
    }
    const std::array<double, 12> & projection_matrix = info->camera_infos[*camera_index].p;
    const int cam_width = static_cast<int>(info->camera_infos[*camera_index].width);
    const int cam_height = static_cast<int>(info->camera_infos[*camera_index].height);

    if (max_detection_cloud_stamp_delta_ > 0.0) {
      const rclcpp::Time det_time(camera_detections.header.stamp, get_clock()->get_clock_type());
      const double stamp_delta_s = std::abs((det_time - cloud_time).seconds());
      if (stamp_delta_s > max_detection_cloud_stamp_delta_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Skipping camera %s: detection–cloud stamp delta %.3f s > %.3f s (relax max_detection_cloud_stamp_delta if "
          "needed)",
          frame_id.c_str(),
          stamp_delta_s,
          max_detection_cloud_stamp_delta_);
        continue;
      }
    }

    geometry_msgs::msg::TransformStamped tf_lidar_to_cam;
    bool have_cached;
    {
      std::lock_guard<std::mutex> lock(transform_cache_mutex_);
      auto it = lidar_to_cam_transform_cache_.find(frame_id);
      if (it != lidar_to_cam_transform_cache_.end()) {
        tf_lidar_to_cam = it->second;
        have_cached = true;
      } else {
        have_cached = false;
      }
    }
    if (!have_cached) {
      try {
        tf_lidar_to_cam = tf_buffer_->lookupTransform(frame_id, lidar_frame_, cloud_time, tf2::durationFromSec(0.05));
      } catch (tf2::TransformException &) {
        try {
          tf_lidar_to_cam = tf_buffer_->lookupTransform(frame_id, lidar_frame_, tf2::TimePointZero);
        } catch (tf2::TransformException & e) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "TF %s -> %s failed: %s. Ensure TF is published (e.g. lidar_cc to camera optical frames).",
            lidar_frame_.c_str(),
            frame_id.c_str(),
            e.what());
          continue;
        }
      }
      std::lock_guard<std::mutex> lock(transform_cache_mutex_);
      lidar_to_cam_transform_cache_[frame_id] = tf_lidar_to_cam;
    }

    std::vector<projection_utils::ClusterCandidate> candidates = cached_candidates;
    auto detection_results = processDetections(
      camera_detections, tf_lidar_to_cam, projection_matrix, cloud, candidates, lidar_header, cam_width, cam_height);

    for (auto & d : detection_results.detections3d.detections) {
      merged_detections3d.detections.push_back(std::move(d));
    }
    for (auto & marker : detection_results.bboxes.markers) {
      marker.ns = frame_id;
      merged_bboxes.markers.push_back(std::move(marker));
    }

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

  if (debug_logging_) {
    RCLCPP_INFO(
      get_logger(),
      "Association result: %zu 3D detection(s), %zu bbox marker(s) from %zu camera(s)",
      merged_detections3d.detections.size(),
      merged_bboxes.markers.size(),
      msg->camera_detections.size());
  }

  if (merged_detections3d.detections.empty() && total_2d > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "No 3D detections produced despite %zu 2D detections. Check: frame_id match, TF lidar_cc->camera, "
      "stamp sync (max_detection_cloud_stamp_delta), and IOU/quality filters.",
      total_2d);
  }

  if (detection_3d_pub_ && detection_3d_pub_->is_activated()) {
    detection_3d_pub_->publish(merged_detections3d);
  }

  if (publish_bounding_box_ && bounding_box_pub_ && bounding_box_pub_->is_activated() && !merged_bboxes.markers.empty())
  {
    bounding_box_pub_->publish(merged_bboxes);
  }
}

DetectionOutputs SpatialAssociationNode::processDetections(
  const vision_msgs::msg::Detection2DArray & detection,
  const geometry_msgs::msg::TransformStamped & transform,
  const std::array<double, 12> & projection_matrix,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  std::vector<projection_utils::ClusterCandidate> candidates,
  const std_msgs::msg::Header & lidar_header,
  int image_width,
  int image_height)
{
  DetectionOutputs detection_outputs;

  if (!cloud || cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Non-ground cloud data not available or empty");
    return detection_outputs;
  }

  if (candidates.empty()) {
    return detection_outputs;
  }

  if (debug_logging_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Camera %s: %zu clusters before IOU filtering, %zu detections",
      transform.header.frame_id.c_str(),
      candidates.size(),
      detection.detections.size());
  }

  projection_utils::assignCandidatesToDetectionsByIOU(
    candidates, detection, transform, projection_matrix, object_detection_confidence_, image_width, image_height);

  if (debug_logging_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Camera %s: %zu clusters kept after IOU filtering",
      transform.header.frame_id.c_str(),
      candidates.size());
  }

  projection_utils::filterCandidatesByClassAwareConstraints(candidates, detection);

  std::vector<pcl::PointIndices> out_indices = projection_utils::extractIndices(candidates);
  auto boxes = core_->computeClusterBoxes(cloud, out_indices);

  if (publish_bounding_box_) {
    detection_outputs.bboxes = projection_utils::computeBoundingBox(boxes, out_indices, lidar_header);
  }

  detection_outputs.detections3d = projection_utils::compute3DDetection(boxes, candidates, lidar_header, detection);

  if (publish_visualization_) {
    detection_outputs.colored_cluster.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    core_->assignClusterColors(cloud, out_indices, detection_outputs.colored_cluster);

    detection_outputs.centroid_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    core_->computeClusterCentroids(cloud, out_indices, detection_outputs.centroid_cloud);
  }
  return detection_outputs;
}

RCLCPP_COMPONENTS_REGISTER_NODE(SpatialAssociationNode)
