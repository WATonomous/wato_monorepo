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
    core_params.density_weight = density_weight_;
    core_params.size_weight = size_weight_;
    core_params.distance_weight = distance_weight_;
    core_params.score_threshold = score_threshold_;
    core_params.merge_threshold = merge_threshold_;
    core_params.object_detection_confidence = object_detection_confidence_;

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

    multi_image_sub_ = this->create_subscription<deep_msgs::msg::MultiImage>(
      kMultiImage, 10, std::bind(&SpatialAssociationNode::multiImageCallback, this, std::placeholders::_1));

    multi_camera_info_sub_ = this->create_subscription<deep_msgs::msg::MultiCameraInfo>(
      kMultiCameraInfo, rclcpp::SensorDataQoS(),
      std::bind(&SpatialAssociationNode::multiCameraInfoCallback, this, std::placeholders::_1));

    dets_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      kDetections, 10, std::bind(&SpatialAssociationNode::detectionCallback, this, std::placeholders::_1));

    // Publishers
    detection_3d_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(kDetection3d, 10);
    bounding_box_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kBoundingBox, 10);
    if (detection_3d_pub_) {
      detection_3d_pub_->on_activate();
    }
    if (bounding_box_pub_) {
      bounding_box_pub_->on_activate();
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
      RCLCPP_INFO(this->get_logger(), "Visualization publishers enabled");
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
  camInfoMap_.clear();
  filtered_point_cloud_.reset();
  cluster_indices.clear();
  working_colored_cluster_.reset();
  working_centroid_cloud_.reset();

  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpatialAssociationNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Shutting down Spatial Association node from state: %s",
    previous_state.label().c_str());

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
  camInfoMap_.clear();
  filtered_point_cloud_.reset();
  cluster_indices.clear();
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
  this->declare_parameter<std::string>("lidar_top_frame", "LIDAR_TOP");

  this->declare_parameter<bool>("publish_visualization", true);
  this->declare_parameter<bool>("debug_logging", false);

  this->declare_parameter<float>("voxel_size", 0.2f);

  this->declare_parameter<double>("euclid_params.cluster_tolerance", 0.5);
  this->declare_parameter<int>("euclid_params.min_cluster_size", 50);
  this->declare_parameter<int>("euclid_params.max_cluster_size", 700);
  this->declare_parameter<bool>("euclid_params.use_adaptive_clustering", true);
  this->declare_parameter<double>("euclid_params.close_threshold", 10.0);
  this->declare_parameter<double>("euclid_params.close_tolerance_mult", 1.5);

  this->declare_parameter<double>("density_filter_params.density_weight", 0.6);
  this->declare_parameter<double>("density_filter_params.size_weight", 0.8);
  this->declare_parameter<double>("density_filter_params.distance_weight", 0.7);
  this->declare_parameter<double>("density_filter_params.score_threshold", 0.6);

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

  publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
  voxel_size_ = static_cast<float>(this->get_parameter("voxel_size").as_double());
  lidar_frame_ = this->get_parameter("lidar_top_frame").as_string();

  euclid_cluster_tolerance_ = this->get_parameter("euclid_params.cluster_tolerance").as_double();
  euclid_min_cluster_size_ = this->get_parameter("euclid_params.min_cluster_size").as_int();
  euclid_max_cluster_size_ = this->get_parameter("euclid_params.max_cluster_size").as_int();
  use_adaptive_clustering_ = this->get_parameter("euclid_params.use_adaptive_clustering").as_bool();
  euclid_close_threshold_ = this->get_parameter("euclid_params.close_threshold").as_double();
  euclid_close_tolerance_mult_ = this->get_parameter("euclid_params.close_tolerance_mult").as_double();

  density_weight_ = this->get_parameter("density_filter_params.density_weight").as_double();
  size_weight_ = this->get_parameter("density_filter_params.size_weight").as_double();
  distance_weight_ = this->get_parameter("density_filter_params.distance_weight").as_double();
  score_threshold_ = this->get_parameter("density_filter_params.score_threshold").as_double();

  merge_threshold_ = this->get_parameter("merge_threshold").as_double();
  object_detection_confidence_ = this->get_parameter("object_detection_confidence").as_double();

  debug_logging_ = this->get_parameter("debug_logging").as_bool();

  if (debug_logging_) {
    RCLCPP_INFO(this->get_logger(), "Debug logging is ENABLED for spatial_association");
  }
  RCLCPP_INFO(this->get_logger(), "Parameters initialized");
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void SpatialAssociationNode::multiCameraInfoCallback(
  const deep_msgs::msg::MultiCameraInfo::SharedPtr msg)
{
  for (const auto & info : msg->camera_infos) {
    const auto & frame_id = info.header.frame_id;
    if (camInfoMap_.find(frame_id) == camInfoMap_.end()) {
      RCLCPP_INFO(this->get_logger(), "Received CameraInfo for '%s' via MultiCameraInfo", frame_id.c_str());
    }
    camInfoMap_[frame_id] = std::make_shared<sensor_msgs::msg::CameraInfo>(info);
  }
}

void SpatialAssociationNode::multiImageCallback(
  const deep_msgs::msg::MultiImage::SharedPtr /*msg*/)
{
  // Camera intrinsics are now received via MultiCameraInfo; nothing to do here
  // beyond keeping the subscription alive so the node stays in the graph.
}

void SpatialAssociationNode::nonGroundCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (debug_logging_) {
    RCLCPP_INFO(this->get_logger(), "Received non-ground cloud with %d points", msg->width * msg->height);
  }

  latest_lidar_msg_ = *msg;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(latest_lidar_msg_, *input_cloud);

  if (input_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty non-ground cloud");
    filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cluster_indices.clear();
    return;
  }

  filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  core_->processPointCloud(input_cloud, filtered_point_cloud_);

  // Pre-compute clusters once per lidar frame so every subsequent
  // Detection2DArray callback reuses the same set.
  core_->performClustering(filtered_point_cloud_, cluster_indices);

  if (debug_logging_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Processed non-ground cloud: %zu points, %zu clusters",
      filtered_point_cloud_->size(),
      cluster_indices.size());
  }
}

void SpatialAssociationNode::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  const auto & frame_id = msg->header.frame_id;

  // Need CameraInfo for this camera
  const auto it = camInfoMap_.find(frame_id);
  if (it == camInfoMap_.end()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "No CameraInfo for '%s', skipping detections", frame_id.c_str());
    return;
  }

  if (!filtered_point_cloud_ || filtered_point_cloud_->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No non-ground cloud data, skipping detection processing");
    return;
  }

  if (cluster_indices.empty()) {
    RCLCPP_DEBUG(get_logger(), "No clusters available");
    return;
  }

  const auto & camera_info = it->second;

  geometry_msgs::msg::TransformStamped tf_cam_to_lidar;
  try {
    tf_cam_to_lidar = tf_buffer_->lookupTransform(frame_id, lidar_frame_, tf2::TimePointZero);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "TF %s->%s failed: %s", lidar_frame_.c_str(), frame_id.c_str(), e.what());
    return;
  }

  auto detection_results = processDetections(*msg, tf_cam_to_lidar, camera_info->p);

  // Publish 3D detections
  if (detection_3d_pub_ && detection_3d_pub_->is_activated()) {
    detection_results.detections3d.header = latest_lidar_msg_.header;
    detection_results.detections3d.header.stamp = this->get_clock()->now();
    detection_3d_pub_->publish(detection_results.detections3d);
  }

  // Publish visualisation topics
  if (publish_visualization_) {
    if (bounding_box_pub_ && bounding_box_pub_->is_activated()) {
      for (auto & marker : detection_results.bboxes.markers) {
        marker.ns = frame_id;
      }
      bounding_box_pub_->publish(detection_results.bboxes);
    }
    if (filtered_lidar_pub_ && filtered_lidar_pub_->is_activated() && detection_results.colored_cluster) {
      sensor_msgs::msg::PointCloud2 pcl2_msg;
      pcl::toROSMsg(*detection_results.colored_cluster, pcl2_msg);
      pcl2_msg.header = latest_lidar_msg_.header;
      filtered_lidar_pub_->publish(pcl2_msg);
    }
    if (cluster_centroid_pub_ && cluster_centroid_pub_->is_activated() && detection_results.centroid_cloud) {
      sensor_msgs::msg::PointCloud2 pcl2_msg;
      pcl::toROSMsg(*detection_results.centroid_cloud, pcl2_msg);
      pcl2_msg.header = latest_lidar_msg_.header;
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
  const std::array<double, 12> & projection_matrix)
{
  DetectionOutputs detection_outputs;

  if (!filtered_point_cloud_ || filtered_point_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Non-ground cloud data not available or empty");
    return detection_outputs;
  }

  if (cluster_indices.empty()) {
    return detection_outputs;
  }

  std::vector<pcl::PointIndices> working_cluster_indices = cluster_indices;

  auto cluster_stats = ProjectionUtils::computeClusterStats(filtered_point_cloud_, working_cluster_indices);

  if (debug_logging_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Camera %s: %zu clusters before IOU filtering, %zu detections",
      transform.child_frame_id.c_str(),
      working_cluster_indices.size(),
      detection.detections.size());
  }

  ProjectionUtils::computeHighestIOUCluster(
    cluster_stats, working_cluster_indices, detection, transform, projection_matrix, object_detection_confidence_);

  if (debug_logging_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Camera %s: %zu clusters kept after IOU filtering",
      transform.child_frame_id.c_str(),
      working_cluster_indices.size());
  }

  auto boxes = core_->computeClusterBoxes(filtered_point_cloud_, working_cluster_indices);

  if (publish_visualization_) {
    detection_outputs.bboxes = ProjectionUtils::computeBoundingBox(boxes, working_cluster_indices, latest_lidar_msg_);
  }

  detection_outputs.detections3d =
    ProjectionUtils::compute3DDetection(boxes, working_cluster_indices, latest_lidar_msg_);

  if (publish_visualization_) {
    detection_outputs.colored_cluster = working_colored_cluster_;
    core_->assignClusterColors(filtered_point_cloud_, working_cluster_indices, detection_outputs.colored_cluster);

    detection_outputs.centroid_cloud = working_centroid_cloud_;
    core_->computeClusterCentroids(filtered_point_cloud_, working_cluster_indices, detection_outputs.centroid_cloud);
  }
  return detection_outputs;
}

RCLCPP_COMPONENTS_REGISTER_NODE(SpatialAssociationNode)
