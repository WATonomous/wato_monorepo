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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "utils/cluster_box_utils.hpp"

namespace
{
/** rclcpp::Time → builtin stamp (same idea as @c to_msg() on newer rclcpp). */
builtin_interfaces::msg::Time timeToMsg(const rclcpp::Time & t)
{
  return static_cast<builtin_interfaces::msg::Time>(t);
}

rclcpp::Time representativeMultiDetectionStamp(
  const deep_msgs::msg::MultiDetection2DArray & msg,
  const rclcpp::Clock::SharedPtr & clock,
  const std::string & fallback_policy)
{
  const auto clock_type = clock->get_clock_type();
  const auto & mh = msg.header.stamp;
  if (mh.sec != 0 || mh.nanosec != 0) {
    return rclcpp::Time(mh, clock_type);
  }
  std::vector<double> cam_secs;
  cam_secs.reserve(msg.camera_detections.size());
  for (const auto & cam : msg.camera_detections) {
    const auto & st = cam.header.stamp;
    if (st.sec == 0 && st.nanosec == 0) {
      continue;
    }
    cam_secs.push_back(rclcpp::Time(st, clock_type).seconds());
  }
  if (cam_secs.empty()) {
    return rclcpp::Time(0, 0, clock_type);
  }
  if (fallback_policy == "max_camera_stamp") {
    const double max_s = *std::max_element(cam_secs.begin(), cam_secs.end());
    return rclcpp::Time(0, 0, clock_type) + rclcpp::Duration::from_seconds(max_s);
  }
  std::sort(cam_secs.begin(), cam_secs.end());
  const size_t n = cam_secs.size();
  const double med_s = (n % 2 == 1) ? cam_secs[n / 2] : 0.5 * (cam_secs[n / 2 - 1] + cam_secs[n / 2]);
  return rclcpp::Time(0, 0, clock_type) + rclcpp::Duration::from_seconds(med_s);
}

struct CrossCameraWorkItem
{
  vision_msgs::msg::Detection3D det3d;
  visualization_msgs::msg::Marker bbox;
  bool has_bbox{false};
  std::vector<int> sorted_indices;
  double score{0.0};
};

void sortUniqueIndicesInPlace(std::vector<int> & v)
{
  std::sort(v.begin(), v.end());
  v.erase(std::unique(v.begin(), v.end()), v.end());
}

size_t sortedIntersectionSize(const std::vector<int> & a, const std::vector<int> & b)
{
  size_t i = 0, j = 0, c = 0;
  while (i < a.size() && j < b.size()) {
    if (a[i] == b[j]) {
      ++c;
      ++i;
      ++j;
    } else if (a[i] < b[j]) {
      ++i;
    } else {
      ++j;
    }
  }
  return c;
}

std::string primaryClassId(const vision_msgs::msg::Detection3D & d)
{
  return d.results.empty() ? std::string() : d.results[0].hypothesis.class_id;
}

double bevDistanceXY(const vision_msgs::msg::Detection3D & a, const vision_msgs::msg::Detection3D & b)
{
  const double dx = a.bbox.center.position.x - b.bbox.center.position.x;
  const double dy = a.bbox.center.position.y - b.bbox.center.position.y;
  return std::hypot(dx, dy);
}

struct Aabb2d
{
  double minx{0.0};
  double maxx{0.0};
  double miny{0.0};
  double maxy{0.0};
};

Aabb2d bevAabbFromBoundingBox3D(const vision_msgs::msg::BoundingBox3D & b)
{
  tf2::Quaternion q;
  tf2::fromMsg(b.center.orientation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double cx = b.center.position.x;
  const double cy = b.center.position.y;
  const double hx = b.size.x * 0.5;
  const double hy = b.size.y * 0.5;
  double minx = std::numeric_limits<double>::infinity();
  double maxx = -std::numeric_limits<double>::infinity();
  double miny = std::numeric_limits<double>::infinity();
  double maxy = -std::numeric_limits<double>::infinity();
  const double corners[4][2] = {{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}};
  for (const auto & corner : corners) {
    const double wx = c * corner[0] - s * corner[1];
    const double wy = s * corner[0] + c * corner[1];
    minx = std::min(minx, cx + wx);
    maxx = std::max(maxx, cx + wx);
    miny = std::min(miny, cy + wy);
    maxy = std::max(maxy, cy + wy);
  }
  return {minx, maxx, miny, maxy};
}

double bevAabbIou(const Aabb2d & a, const Aabb2d & b)
{
  const double ix0 = std::max(a.minx, b.minx);
  const double ix1 = std::min(a.maxx, b.maxx);
  const double iy0 = std::max(a.miny, b.miny);
  const double iy1 = std::min(a.maxy, b.maxy);
  if (ix1 <= ix0 || iy1 <= iy0) return 0.0;
  const double inter = (ix1 - ix0) * (iy1 - iy0);
  const double ua = std::max(0.0, a.maxx - a.minx) * std::max(0.0, a.maxy - a.miny);
  const double ub = std::max(0.0, b.maxx - b.minx) * std::max(0.0, b.maxy - b.miny);
  const double uni = ua + ub - inter;
  return uni > 1e-9 ? inter / uni : 0.0;
}

bool areCrossCameraDuplicates(
  const CrossCameraWorkItem & a,
  const CrossCameraWorkItem & b,
  double min_overlap,
  double weak_overlap,
  double max_center_m,
  double min_bev_iou,
  double min_bev_iou_no_class)
{
  const std::string ca = primaryClassId(a.det3d);
  const std::string cb = primaryClassId(b.det3d);

  if (!a.sorted_indices.empty() && !b.sorted_indices.empty()) {
    const size_t inter = sortedIntersectionSize(a.sorted_indices, b.sorted_indices);
    const size_t mn = std::min(a.sorted_indices.size(), b.sorted_indices.size());
    if (mn > 0) {
      const double ol = static_cast<double>(inter) / static_cast<double>(mn);
      if (ol >= min_overlap) return true;
      if (ol >= weak_overlap && !ca.empty() && ca == cb && bevDistanceXY(a.det3d, b.det3d) <= max_center_m) {
        return true;
      }
    }
  }

  const Aabb2d ra = bevAabbFromBoundingBox3D(a.det3d.bbox);
  const Aabb2d rb = bevAabbFromBoundingBox3D(b.det3d.bbox);
  const double biou = bevAabbIou(ra, rb);
  if (!ca.empty() && ca == cb && biou >= min_bev_iou) return true;
  if (ca.empty() && cb.empty() && biou >= min_bev_iou_no_class) return true;
  return false;
}

void deduplicateCrossCameraWorkItems(
  std::vector<CrossCameraWorkItem> & items,
  double min_overlap,
  double weak_overlap,
  double max_center_m,
  double min_bev_iou,
  double min_bev_iou_no_class)
{
  if (items.size() <= 1u) return;

  std::vector<size_t> order(items.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](size_t i, size_t j) { return items[i].score > items[j].score; });

  std::vector<char> suppressed(items.size(), 0);
  for (size_t oi = 0; oi < order.size(); ++oi) {
    const size_t i = order[oi];
    if (suppressed[i]) continue;
    for (size_t j = 0; j < items.size(); ++j) {
      if (i == j || suppressed[j]) continue;
      if (areCrossCameraDuplicates(
            items[i], items[j], min_overlap, weak_overlap, max_center_m, min_bev_iou, min_bev_iou_no_class))
      {
        suppressed[j] = 1;
      }
    }
  }

  std::vector<CrossCameraWorkItem> kept;
  kept.reserve(items.size());
  for (size_t i = 0; i < items.size(); ++i) {
    if (!suppressed[i]) kept.push_back(std::move(items[i]));
  }
  items = std::move(kept);
}
}  // namespace

SpatialAssociationNode::SpatialAssociationNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("spatial_association_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Spatial Association lifecycle node created");
}

SpatialAssociationNode::~SpatialAssociationNode()
{
  stopAssociationWorker();
  stopWorker();
}

void SpatialAssociationNode::invalidateClusteredCloudCacheLocked()
{
  clustered_cache_invalidation_generation_.fetch_add(1, std::memory_order_release);
  clustered_cloud_cache_.clear();
}

void SpatialAssociationNode::resetCachesAfterWorkerStopped()
{
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    invalidateClusteredCloudCacheLocked();
  }
  {
    std::lock_guard<std::mutex> lock(transform_cache_mutex_);
    lidar_to_cam_transform_cache_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(latest_detections_mutex_);
    latest_detections_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(publish_freshness_mutex_);
    last_published_detection_seq_ = 0;
    last_published_clustered_seq_ = 0;
    last_association_publish_pair_valid_ = false;
    last_association_publish_det_ns_ = 0;
    last_association_publish_cloud_ns_ = 0;
  }
}

void SpatialAssociationNode::notifyAssociationWorker(std::shared_ptr<const ClusteredCloudSnapshot> forced_snapshot_hint)
{
  std::lock_guard<std::mutex> lock(association_trigger_mutex_);
  if (forced_snapshot_hint) {
    association_forced_snapshot_hint_ = std::move(forced_snapshot_hint);
  }
  association_trigger_pending_ = true;
  association_trigger_cv_.notify_one();
}

void SpatialAssociationNode::stopAssociationWorker()
{
  {
    std::lock_guard<std::mutex> lock(association_trigger_mutex_);
    association_shutdown_ = true;
    association_trigger_pending_ = true;
    association_forced_snapshot_hint_.reset();
    association_trigger_cv_.notify_all();
  }
  if (association_thread_.joinable()) {
    association_thread_.join();
  }
}

void SpatialAssociationNode::associationLoop()
{
  while (true) {
    std::shared_ptr<const ClusteredCloudSnapshot> forced_snapshot_hint;
    {
      std::unique_lock<std::mutex> lock(association_trigger_mutex_);
      association_trigger_cv_.wait(lock, [this] { return association_trigger_pending_; });
      if (association_shutdown_) {
        break;
      }
      association_trigger_pending_ = false;
      forced_snapshot_hint = std::move(association_forced_snapshot_hint_);
      association_forced_snapshot_hint_.reset();
    }

    deep_msgs::msg::MultiDetection2DArray::SharedPtr dets;
    {
      std::lock_guard<std::mutex> lock(latest_detections_mutex_);
      dets = latest_detections_;
    }
    if (!dets || dets->camera_detections.empty()) {
      continue;
    }
    runAssociationFromDetections(dets, std::move(forced_snapshot_hint));
  }
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
    core_params.max_lidar_range_m = quality_max_distance_;

    // Load multi-band clustering config if present.
    core_params.clustering_bands = clustering_bands_;

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
    auto cloud_qos = rclcpp::QoS(10);
    cloud_qos.reliable();
    cloud_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    non_ground_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      kNonGroundCloud,
      cloud_qos,
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
      det3d_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
        kDetection3DInput,
        det_qos,
        std::bind(&SpatialAssociationNode::detection3DCallback, this, std::placeholders::_1));
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
    {
      std::lock_guard<std::mutex> lock(association_trigger_mutex_);
      association_shutdown_ = false;
      association_trigger_pending_ = false;
      association_forced_snapshot_hint_.reset();
    }
    association_thread_ = std::thread(&SpatialAssociationNode::associationLoop, this);
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
  det3d_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(det3d_mutex_);
    latest_det3d_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_multi_image_.reset();
  }

  stopAssociationWorker();
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
  det3d_sub_.reset();
  stopAssociationWorker();
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
    std::lock_guard<std::mutex> lock(det3d_mutex_);
    latest_det3d_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_multi_image_.reset();
  }
  resetCachesAfterWorkerStopped();

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
  det3d_sub_.reset();
  stopAssociationWorker();
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
    std::lock_guard<std::mutex> lock(det3d_mutex_);
    latest_det3d_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_multi_image_.reset();
  }
  resetCachesAfterWorkerStopped();

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
  declareIfMissing(this, "max_detection_cloud_stamp_delta", 0.08);
  declareIfMissing(this, "detection_stamp_fallback_policy", std::string("median_camera_stamp"));
  declareIfMissing(this, "clustered_cloud_cache_size", 24);
  declareIfMissing(this, "raw_cloud_pending_queue_size", 1);
  declareIfMissing(this, "raw_cloud_latest_only", true);
  declareIfMissing(this, "bbox_markers_use_detection_stamp", false);
  declareIfMissing(this, "skip_repeat_association_publish", true);

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

  // Multi-band clustering: parallel arrays. Empty = use legacy two-band or flat clustering.
  declareIfMissing(this, "euclid_params.clustering_bands.max_distances", std::vector<double>{});
  declareIfMissing(this, "euclid_params.clustering_bands.tolerance_mults", std::vector<double>{});
  declareIfMissing(this, "euclid_params.clustering_bands.min_cluster_sizes", std::vector<int64_t>{});

  declareIfMissing(this, "merge_threshold", 0.3);

  declareIfMissing(this, "cross_camera_dedup.enabled", true);
  declareIfMissing(this, "cross_camera_dedup.min_lidar_index_overlap", 0.5);
  declareIfMissing(this, "cross_camera_dedup.weak_lidar_index_overlap", 0.22);
  declareIfMissing(this, "cross_camera_dedup.max_center_dist_m", 1.8);
  declareIfMissing(this, "cross_camera_dedup.min_bev_box_iou", 0.16);
  declareIfMissing(this, "cross_camera_dedup.min_bev_box_iou_no_class", 0.30);

  declareIfMissing(this, "object_detection_confidence", 0.4f);

  declareIfMissing(this, "quality_filter_params.max_distance", 60.0);
  declareIfMissing(this, "quality_filter_params.min_points", 5);
  declareIfMissing(this, "quality_filter_params.min_height", 0.15);
  declareIfMissing(this, "quality_filter_params.min_points_default", 8);
  declareIfMissing(this, "quality_filter_params.min_points_far", 8);
  declareIfMissing(this, "quality_filter_params.min_points_medium", 10);
  declareIfMissing(this, "quality_filter_params.min_points_large", 30);
  declareIfMissing(this, "quality_filter_params.distance_threshold_far", 30.0);
  declareIfMissing(this, "quality_filter_params.distance_threshold_medium", 20.0);
  declareIfMissing(this, "quality_filter_params.volume_threshold_large", 8.0);
  declareIfMissing(this, "quality_filter_params.min_density", 5.0);
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
  declareIfMissing(this, pu + "use_l_shaped_fitting", true);
  declareIfMissing(this, pu + "min_camera_z_distance", 1.0);
  declareIfMissing(this, pu + "detection_score_weight", 0.6);
  declareIfMissing(this, pu + "iou_score_weight", 0.4);
  declareIfMissing(this, pu + "image_width", 1280);
  declareIfMissing(this, pu + "image_height", 1024);
  declareIfMissing(this, pu + "association_strict_matching", true);
  declareIfMissing(this, pu + "association_centroid_inner_box_fraction", 0.85);
  declareIfMissing(this, pu + "association_min_inside_point_fraction", 0.45);
  declareIfMissing(this, pu + "association_min_ar_consistency_score", 0.30);
  declareIfMissing(this, pu + "association_use_point_projection_rect_for_iou", true);
  // Association scoring weights.
  declareIfMissing(this, pu + "association_weight_iou", 0.30);
  declareIfMissing(this, pu + "association_weight_inside_fraction", 0.28);
  declareIfMissing(this, pu + "association_weight_ar", 0.18);
  declareIfMissing(this, pu + "association_weight_centroid", 0.14);
  declareIfMissing(this, pu + "association_weight_points", 0.10);
  // Class-aware threshold scaling.
  declareIfMissing(this, pu + "person_inside_fraction_scale", 0.85);
  declareIfMissing(this, pu + "vehicle_inside_fraction_scale", 0.85);
  declareIfMissing(this, pu + "person_iou_threshold_scale", 0.90);
  declareIfMissing(this, pu + "vehicle_iou_threshold_scale", 0.90);
  declareIfMissing(this, pu + "person_ar_consistency_scale", 0.70);
  declareIfMissing(this, pu + "association_min_inside_points", 2);
  declareIfMissing(this, pu + "min_size_consistency_score", 0.25);
  declareIfMissing(this, pu + "min_det_area_far_px2", 400.0);
  // Scoring internals.
  declareIfMissing(this, pu + "centroid_score_detection_scale", 0.35);
  declareIfMissing(this, pu + "point_score_saturation_count", 120.0);
  declareIfMissing(this, pu + "association_centroid_distance_multiplier", 1.5);
  declareIfMissing(this, pu + "single_point_bbox_margin_px", 3.0);
  declareIfMissing(this, pu + "min_depth_for_enrichment", 0.1);
  // Orientation / L-shape search.
  declareIfMissing(this, pu + "l_shape_energy_variance_weight", 0.5);
  declareIfMissing(this, pu + "orientation_coarse_step_multiplier", 5.0);
  declareIfMissing(this, pu + "orientation_fine_range_multiplier", 2.5);

  declareIfMissing(this, pu + "depth_score_weight", 0.15);
  declareIfMissing(this, pu + "depth_score_scale", 5.0);
  // Class-aware size priors (parallel arrays, same length).
  declareIfMissing(this, pu + "enable_size_prior", true);
  declareIfMissing(this, pu + "size_prior_weight", 0.10);
  declareIfMissing(this, pu + "size_prior_scale", 0.5);
  declareIfMissing(this, pu + "size_prior_classes", std::vector<std::string>{});
  declareIfMissing(this, pu + "size_prior_min_widths", std::vector<double>{});
  declareIfMissing(this, pu + "size_prior_max_widths", std::vector<double>{});
  declareIfMissing(this, pu + "size_prior_min_lengths", std::vector<double>{});
  declareIfMissing(this, pu + "size_prior_max_lengths", std::vector<double>{});
  declareIfMissing(this, pu + "size_prior_min_heights", std::vector<double>{});
  declareIfMissing(this, pu + "size_prior_max_heights", std::vector<double>{});
  declareIfMissing(this, pu + "use_hungarian_assignment", true);
  declareIfMissing(this, pu + "far_iou_threshold_scale", 0.5);
  declareIfMissing(this, pu + "medium_iou_threshold_scale", 0.75);
  declareIfMissing(this, pu + "far_inside_fraction_scale", 0.6);
  declareIfMissing(this, pu + "medium_inside_fraction_scale", 0.8);
  declareIfMissing(this, pu + "quality_person_max_height_m", 2.5);
  declareIfMissing(this, pu + "quality_person_max_footprint_xy_m", 1.45);
  declareIfMissing(this, pu + "quality_person_max_volume_m3", 4.0);
  declareIfMissing(this, pu + "quality_vehicle_max_height_m", 4.8);

  publish_bounding_box_ = this->get_parameter("publish_bounding_box").as_bool();
  publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
  publish_image_visualization_ = this->get_parameter("publish_image_visualization").as_bool();
  voxel_size_ = static_cast<float>(this->get_parameter("voxel_size").as_double());
  lidar_frame_ = this->get_parameter("lidar_frame").as_string();
  max_detection_cloud_stamp_delta_ = this->get_parameter("max_detection_cloud_stamp_delta").as_double();
  detection_stamp_fallback_policy_ = this->get_parameter("detection_stamp_fallback_policy").as_string();
  clustered_cloud_cache_size_ = static_cast<size_t>(
    std::max<int64_t>(static_cast<int64_t>(1), this->get_parameter("clustered_cloud_cache_size").as_int()));
  pending_cloud_queue_max_ = static_cast<size_t>(
    std::max<int64_t>(static_cast<int64_t>(1), this->get_parameter("raw_cloud_pending_queue_size").as_int()));
  raw_cloud_latest_only_ = this->get_parameter("raw_cloud_latest_only").as_bool();
  bbox_markers_use_detection_stamp_ = this->get_parameter("bbox_markers_use_detection_stamp").as_bool();
  skip_repeat_association_publish_ = this->get_parameter("skip_repeat_association_publish").as_bool();

  euclid_cluster_tolerance_ = this->get_parameter("euclid_params.cluster_tolerance").as_double();
  euclid_min_cluster_size_ = this->get_parameter("euclid_params.min_cluster_size").as_int();
  euclid_max_cluster_size_ = this->get_parameter("euclid_params.max_cluster_size").as_int();
  use_adaptive_clustering_ = this->get_parameter("euclid_params.use_adaptive_clustering").as_bool();
  euclid_close_threshold_ = this->get_parameter("euclid_params.close_threshold").as_double();
  euclid_close_tolerance_mult_ = this->get_parameter("euclid_params.close_tolerance_mult").as_double();

  // Load multi-band clustering config.
  {
    auto band_dists = this->get_parameter("euclid_params.clustering_bands.max_distances").as_double_array();
    auto band_mults = this->get_parameter("euclid_params.clustering_bands.tolerance_mults").as_double_array();
    auto band_mins = this->get_parameter("euclid_params.clustering_bands.min_cluster_sizes").as_integer_array();
    clustering_bands_.clear();
    if (!band_dists.empty() && band_dists.size() == band_mults.size() && band_dists.size() == band_mins.size()) {
      for (size_t i = 0; i < band_dists.size(); ++i) {
        clustering_bands_.push_back({band_dists[i], band_mults[i], static_cast<int>(band_mins[i])});
      }
    }
  }

  merge_threshold_ = this->get_parameter("merge_threshold").as_double();
  cross_camera_dedup_enabled_ = this->get_parameter("cross_camera_dedup.enabled").as_bool();
  cross_camera_min_lidar_overlap_ = this->get_parameter("cross_camera_dedup.min_lidar_index_overlap").as_double();
  cross_camera_weak_lidar_overlap_ = this->get_parameter("cross_camera_dedup.weak_lidar_index_overlap").as_double();
  cross_camera_max_center_dist_m_ = this->get_parameter("cross_camera_dedup.max_center_dist_m").as_double();
  cross_camera_min_bev_box_iou_ = this->get_parameter("cross_camera_dedup.min_bev_box_iou").as_double();
  cross_camera_min_bev_box_iou_no_class_ =
    this->get_parameter("cross_camera_dedup.min_bev_box_iou_no_class").as_double();
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
  proj_params.use_l_shaped_fitting = this->get_parameter(pu + "use_l_shaped_fitting").as_bool();
  proj_params.min_camera_z_distance = this->get_parameter(pu + "min_camera_z_distance").as_double();
  proj_params.detection_score_weight = this->get_parameter(pu + "detection_score_weight").as_double();
  proj_params.iou_score_weight = this->get_parameter(pu + "iou_score_weight").as_double();
  proj_params.image_width = this->get_parameter(pu + "image_width").as_int();
  proj_params.image_height = this->get_parameter(pu + "image_height").as_int();
  proj_params.association_strict_matching = this->get_parameter(pu + "association_strict_matching").as_bool();
  proj_params.association_centroid_inner_box_fraction =
    this->get_parameter(pu + "association_centroid_inner_box_fraction").as_double();
  proj_params.association_min_inside_point_fraction =
    this->get_parameter(pu + "association_min_inside_point_fraction").as_double();
  proj_params.association_min_ar_consistency_score =
    this->get_parameter(pu + "association_min_ar_consistency_score").as_double();
  proj_params.association_use_point_projection_rect_for_iou =
    this->get_parameter(pu + "association_use_point_projection_rect_for_iou").as_bool();
  // Association scoring weights.
  proj_params.association_weight_iou = this->get_parameter(pu + "association_weight_iou").as_double();
  proj_params.association_weight_inside_fraction =
    this->get_parameter(pu + "association_weight_inside_fraction").as_double();
  proj_params.association_weight_ar = this->get_parameter(pu + "association_weight_ar").as_double();
  proj_params.association_weight_centroid = this->get_parameter(pu + "association_weight_centroid").as_double();
  proj_params.association_weight_points = this->get_parameter(pu + "association_weight_points").as_double();
  // Class-aware threshold scaling.
  proj_params.person_inside_fraction_scale = this->get_parameter(pu + "person_inside_fraction_scale").as_double();
  proj_params.vehicle_inside_fraction_scale = this->get_parameter(pu + "vehicle_inside_fraction_scale").as_double();
  proj_params.person_iou_threshold_scale = this->get_parameter(pu + "person_iou_threshold_scale").as_double();
  proj_params.vehicle_iou_threshold_scale = this->get_parameter(pu + "vehicle_iou_threshold_scale").as_double();
  proj_params.person_ar_consistency_scale = this->get_parameter(pu + "person_ar_consistency_scale").as_double();
  proj_params.association_min_inside_points = this->get_parameter(pu + "association_min_inside_points").as_int();
  proj_params.min_size_consistency_score = this->get_parameter(pu + "min_size_consistency_score").as_double();
  proj_params.min_det_area_far_px2 = this->get_parameter(pu + "min_det_area_far_px2").as_double();
  // Scoring internals.
  proj_params.centroid_score_detection_scale = this->get_parameter(pu + "centroid_score_detection_scale").as_double();
  proj_params.point_score_saturation_count = this->get_parameter(pu + "point_score_saturation_count").as_double();
  proj_params.association_centroid_distance_multiplier =
    this->get_parameter(pu + "association_centroid_distance_multiplier").as_double();
  proj_params.single_point_bbox_margin_px = this->get_parameter(pu + "single_point_bbox_margin_px").as_double();
  min_depth_for_enrichment_ = this->get_parameter(pu + "min_depth_for_enrichment").as_double();
  proj_params.min_depth_for_enrichment = min_depth_for_enrichment_;
  // Orientation / L-shape search.
  proj_params.l_shape_energy_variance_weight = this->get_parameter(pu + "l_shape_energy_variance_weight").as_double();
  proj_params.orientation_coarse_step_multiplier =
    this->get_parameter(pu + "orientation_coarse_step_multiplier").as_double();
  proj_params.orientation_fine_range_multiplier =
    this->get_parameter(pu + "orientation_fine_range_multiplier").as_double();

  proj_params.depth_score_weight = this->get_parameter(pu + "depth_score_weight").as_double();
  proj_params.depth_score_scale = this->get_parameter(pu + "depth_score_scale").as_double();
  // Class-aware size priors.
  proj_params.enable_size_prior = this->get_parameter(pu + "enable_size_prior").as_bool();
  proj_params.size_prior_weight = this->get_parameter(pu + "size_prior_weight").as_double();
  proj_params.size_prior_scale = this->get_parameter(pu + "size_prior_scale").as_double();
  {
    auto sp_cls = this->get_parameter(pu + "size_prior_classes").as_string_array();
    auto sp_min_w = this->get_parameter(pu + "size_prior_min_widths").as_double_array();
    auto sp_max_w = this->get_parameter(pu + "size_prior_max_widths").as_double_array();
    auto sp_min_l = this->get_parameter(pu + "size_prior_min_lengths").as_double_array();
    auto sp_max_l = this->get_parameter(pu + "size_prior_max_lengths").as_double_array();
    auto sp_min_h = this->get_parameter(pu + "size_prior_min_heights").as_double_array();
    auto sp_max_h = this->get_parameter(pu + "size_prior_max_heights").as_double_array();
    const size_t n = sp_cls.size();
    if (
      n > 0 && sp_min_w.size() == n && sp_max_w.size() == n && sp_min_l.size() == n && sp_max_l.size() == n &&
      sp_min_h.size() == n && sp_max_h.size() == n)
    {
      for (size_t i = 0; i < n; ++i) {
        proj_params.size_priors[sp_cls[i]] = {
          sp_min_w[i], sp_max_w[i], sp_min_l[i], sp_max_l[i], sp_min_h[i], sp_max_h[i]};
      }
    } else if (n > 0) {
      RCLCPP_WARN(
        this->get_logger(), "size_prior arrays have mismatched lengths (%zu classes); size priors disabled", n);
    }
  }
  proj_params.use_hungarian_assignment = this->get_parameter(pu + "use_hungarian_assignment").as_bool();
  proj_params.far_iou_threshold_scale = this->get_parameter(pu + "far_iou_threshold_scale").as_double();
  proj_params.medium_iou_threshold_scale = this->get_parameter(pu + "medium_iou_threshold_scale").as_double();
  proj_params.far_inside_fraction_scale = this->get_parameter(pu + "far_inside_fraction_scale").as_double();
  proj_params.medium_inside_fraction_scale = this->get_parameter(pu + "medium_inside_fraction_scale").as_double();
  proj_params.quality_person_max_height_m =
    static_cast<float>(this->get_parameter(pu + "quality_person_max_height_m").as_double());
  proj_params.quality_person_max_footprint_xy_m =
    static_cast<float>(this->get_parameter(pu + "quality_person_max_footprint_xy_m").as_double());
  proj_params.quality_person_max_volume_m3 =
    static_cast<float>(this->get_parameter(pu + "quality_person_max_volume_m3").as_double());
  proj_params.quality_vehicle_max_height_m =
    static_cast<float>(this->get_parameter(pu + "quality_vehicle_max_height_m").as_double());

  proj_params.quality_max_distance = this->get_parameter("quality_filter_params.max_distance").as_double();
  quality_max_distance_ = proj_params.quality_max_distance;
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
    pending_cloud_queue_.clear();
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
    uint64_t my_seq = 0;
    uint64_t gen_at_dequeue = 0;
    {
      std::unique_lock<std::mutex> lock(job_mutex_);
      job_cv_.wait(lock, [this] { return worker_shutdown_ || !pending_cloud_queue_.empty(); });
      if (worker_shutdown_) {
        break;
      }
      PendingRawCloudJob job = std::move(pending_cloud_queue_.front());
      pending_cloud_queue_.pop_front();
      gen_at_dequeue = clustered_cache_invalidation_generation_.load(std::memory_order_acquire);
      my_cloud = std::move(job.cloud);
      my_header = job.header;
      my_seq = job.seq;
    }
    const auto is_stale_latest_only = [this, my_seq]() {
      return raw_cloud_latest_only_ && my_seq != latest_raw_cloud_seq_.load(std::memory_order_acquire);
    };
    if (is_stale_latest_only()) {
      continue;
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
    if (is_stale_latest_only()) {
      continue;
    }
    if (worker_shutdown_) {
      break;
    }
    std::vector<pcl::PointIndices> indices;
    core_->performClustering(filtered, indices);
    if (is_stale_latest_only()) {
      continue;
    }
    const size_t num_points = filtered->size();
    const size_t num_clusters = indices.size();

    auto snap = std::make_shared<ClusteredCloudSnapshot>();
    snap->seq = my_seq;
    snap->header = my_header;
    snap->cloud_stamp = rclcpp::Time(my_header.stamp, get_clock()->get_clock_type());
    snap->cloud = std::move(filtered);
    snap->cluster_indices = std::move(indices);
    snap->global_candidates = projection_utils::buildCandidates(snap->cloud, snap->cluster_indices);

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (gen_at_dequeue != clustered_cache_invalidation_generation_.load(std::memory_order_acquire)) {
        if (debug_logging_) {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *get_clock(),
            2000,
            "Dropped clustered LiDAR snapshot: cache was invalidated while processing");
        }
        continue;
      }
      if (is_stale_latest_only()) {
        continue;
      }
      snap->seq = latest_clustered_cloud_seq_.fetch_add(1, std::memory_order_acq_rel) + 1;
      clustered_cloud_cache_.push_back(snap);
      while (clustered_cloud_cache_.size() > clustered_cloud_cache_size_) {
        clustered_cloud_cache_.pop_front();
      }
    }
    if (is_stale_latest_only()) {
      continue;
    }
    notifyAssociationWorker(snap);
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
    ++cloud_job_id_;
    invalidateClusteredCloudCacheLocked();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (raw_cloud_latest_only_) {
      pending_cloud_queue_.clear();
    } else {
      while (pending_cloud_queue_.size() >= pending_cloud_queue_max_) {
        pending_cloud_queue_.pop_front();
      }
    }
    PendingRawCloudJob job;
    job.cloud = std::move(input_cloud);
    job.header = msg->header;
    job.seq = latest_raw_cloud_seq_.fetch_add(1, std::memory_order_acq_rel) + 1;
    pending_cloud_queue_.push_back(std::move(job));
    ++cloud_job_id_;
    job_cv_.notify_one();
  }
}

std::shared_ptr<SpatialAssociationNode::ClusteredCloudSnapshot> SpatialAssociationNode::pickNearestClusteredCloudLocked(
  const rclcpp::Time & det_time)
{
  std::shared_ptr<ClusteredCloudSnapshot> best;
  double best_abs_s = std::numeric_limits<double>::infinity();
  for (const auto & snap : clustered_cloud_cache_) {
    if (!snap || !snap->cloud || snap->cloud->empty()) {
      continue;
    }
    const double d = std::abs((det_time - snap->cloud_stamp).seconds());
    if (max_detection_cloud_stamp_delta_ > 0.0 && d > max_detection_cloud_stamp_delta_) {
      continue;
    }
    if (d < best_abs_s) {
      best_abs_s = d;
      best = snap;
    }
  }
  return best;
}

void SpatialAssociationNode::multiDetectionCallback(const deep_msgs::msg::MultiDetection2DArray::SharedPtr msg)
{
  if (msg->camera_detections.empty()) {
    {
      std::lock_guard<std::mutex> lock(latest_detections_mutex_);
      latest_detections_.reset();
    }
    {
      std::lock_guard<std::mutex> lock(publish_freshness_mutex_);
      last_published_detection_seq_ = 0;
      last_published_clustered_seq_ = 0;
      last_association_publish_pair_valid_ = false;
      last_association_publish_det_ns_ = 0;
      last_association_publish_cloud_ns_ = 0;
    }
    if (publish_bounding_box_ && bounding_box_pub_ && bounding_box_pub_->is_activated()) {
      visualization_msgs::msg::MarkerArray clear_markers;
      visualization_msgs::msg::Marker delete_all;
      delete_all.header.frame_id = lidar_frame_;
      delete_all.header.stamp = timeToMsg(get_clock()->now());
      delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
      clear_markers.markers.push_back(std::move(delete_all));
      bounding_box_pub_->publish(clear_markers);
    }
    return;
  }
  {
    std::lock_guard<std::mutex> lock(latest_detections_mutex_);
    latest_detections_ = msg;
    latest_detection_seq_.fetch_add(1, std::memory_order_acq_rel);
  }
  notifyAssociationWorker();
}

void SpatialAssociationNode::detection3DCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(det3d_mutex_);
  latest_det3d_ = msg;
}

void SpatialAssociationNode::runAssociationFromDetections(
  const deep_msgs::msg::MultiDetection2DArray::SharedPtr msg,
  std::shared_ptr<const ClusteredCloudSnapshot> forced_snapshot)
{
  std::lock_guard<std::mutex> assoc_lock(association_mutex_);
  if (!msg || msg->camera_detections.empty()) {
    return;
  }
  const bool cloud_triggered =
    static_cast<bool>(forced_snapshot && forced_snapshot->cloud && !forced_snapshot->cloud->empty());
  if (!cloud_triggered && raw_cloud_latest_only_) {
    const uint64_t raw_seq = latest_raw_cloud_seq_.load(std::memory_order_acquire);
    const uint64_t clustered_seq = latest_clustered_cloud_seq_.load(std::memory_order_acquire);
    if (raw_seq > clustered_seq) {
      return;
    }
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

  const rclcpp::Time rep_det_time =
    representativeMultiDetectionStamp(*msg, get_clock(), detection_stamp_fallback_policy_);
  std::shared_ptr<const ClusteredCloudSnapshot> snap;
  bool cache_was_empty = false;
  if (forced_snapshot && forced_snapshot->cloud && !forced_snapshot->cloud->empty()) {
    const double forced_dt_s = std::abs((rep_det_time - forced_snapshot->cloud_stamp).seconds());
    const bool forced_within_delta =
      (max_detection_cloud_stamp_delta_ <= 0.0) || (forced_dt_s <= max_detection_cloud_stamp_delta_);
    if (forced_within_delta) {
      snap = std::move(forced_snapshot);
    }
  }
  if (!snap) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cache_was_empty = clustered_cloud_cache_.empty();
    snap = pickNearestClusteredCloudLocked(rep_det_time);
  }

  if (!snap || !snap->cloud || snap->cloud->empty()) {
    if (!cache_was_empty && max_detection_cloud_stamp_delta_ > 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No clustered LiDAR snapshot within max_detection_cloud_stamp_delta (%.3f s) of detection time — skipping "
        "batch (increase delta or clustered_cloud_cache_size if LiDAR lags)",
        max_detection_cloud_stamp_delta_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No non-ground cloud data, skipping detection processing");
    }
    return;
  }

  const uint64_t association_detection_seq = latest_detection_seq_.load(std::memory_order_acquire);
  const uint64_t association_clustered_seq = snap->seq;

  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud = snap->cloud;
  const std::vector<pcl::PointIndices> & indices = snap->cluster_indices;
  const std_msgs::msg::Header & lidar_header = snap->header;

  if (indices.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No clusters available from non-ground cloud");
    return;
  }

  if (!snap->global_candidates.has_value() || snap->global_candidates->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No cluster candidates available");
    return;
  }

  const rclcpp::Time & cloud_time = snap->cloud_stamp;

  // Grab latest enriched 3D detections (flattened across cameras, 1:1 with 2D)
  vision_msgs::msg::Detection3DArray::SharedPtr det3d_msg;
  {
    std::lock_guard<std::mutex> lock(det3d_mutex_);
    det3d_msg = latest_det3d_;
  }

  // Transform from the 3D detection frame (global) to LiDAR frame for depth comparison
  std::optional<geometry_msgs::msg::TransformStamped> tf_det3d_to_lidar;
  if (det3d_msg && !det3d_msg->detections.empty()) {
    const std::string & det3d_frame = det3d_msg->header.frame_id;
    if (!det3d_frame.empty() && det3d_frame != lidar_frame_) {
      try {
        tf_det3d_to_lidar = tf_buffer_->lookupTransform(lidar_frame_, det3d_frame, tf2::TimePointZero);
      } catch (tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "TF %s -> %s failed for enriched 3D detections: %s — depth scoring disabled this cycle",
          det3d_frame.c_str(),
          lidar_frame_.c_str(),
          e.what());
        det3d_msg.reset();
      }
    }
    // If det3d_frame == lidar_frame_, no transform needed (tf_det3d_to_lidar stays nullopt)
  }

  vision_msgs::msg::Detection3DArray merged_detections3d;
  // LiDAR snapshot time; if bbox_markers_use_detection_stamp_ is true, MarkerArray headers may differ (viz only).
  merged_detections3d.header = lidar_header;
  visualization_msgs::msg::MarkerArray merged_bboxes;

  // Traffic light 3D detections to append directly (no LiDAR association needed)
  std::vector<vision_msgs::msg::Detection3D> traffic_light_passthrough;

  std::vector<CrossCameraWorkItem> accumulated;
  accumulated.reserve(total_2d);

  size_t skipped_no_camera_info = 0;
  size_t skipped_stamp_delta = 0;
  size_t skipped_tf_failed = 0;
  size_t cameras_with_2d_but_zero_3d = 0;

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
      ++skipped_no_camera_info;
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
        ++skipped_stamp_delta;
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
        // Static extrinsics: TimePointZero only (consistent with frame_id-keyed cache).
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
        ++skipped_tf_failed;
        continue;
      }
      std::lock_guard<std::mutex> lock(transform_cache_mutex_);
      lidar_to_cam_transform_cache_[frame_id] = tf_lidar_to_cam;
    }

    std::vector<projection_utils::ClusterCandidate> candidates = *snap->global_candidates;

    // Separate traffic lights (bypass LiDAR) and build depth vector for remaining detections
    std::vector<std::optional<double>> cam_depths;
    const size_t num_cam_dets = camera_detections.detections.size();
    // filtered_dets excludes traffic lights; cam_depths is indexed to match filtered_dets
    vision_msgs::msg::Detection2DArray filtered_dets;
    filtered_dets.header = camera_detections.header;
    bool has_traffic_lights = false;

    // Match each 2D detection to its enriched 3D detection by comparing primary hypothesis
    // (class_id + score are identical since the attribute assigner copies results directly)
    for (size_t i = 0; i < num_cam_dets; ++i) {
      const auto & det2d = camera_detections.detections[i];
      const std::string cls2d = det2d.results.empty() ? std::string() : det2d.results[0].hypothesis.class_id;
      const double score2d = det2d.results.empty() ? 0.0 : det2d.results[0].hypothesis.score;

      // Find matching 3D detection by class_id + score
      const vision_msgs::msg::Detection3D * matched_3d = nullptr;
      if (det3d_msg) {
        for (const auto & det3d : det3d_msg->detections) {
          if (
            !det3d.results.empty() && det3d.results[0].hypothesis.class_id == cls2d &&
            std::abs(det3d.results[0].hypothesis.score - score2d) < 1e-6)
          {
            matched_3d = &det3d;
            break;
          }
        }
      }

      if (matched_3d) {
        const std::string & cls3d = matched_3d->results[0].hypothesis.class_id;
        if (cls3d == "traffic light") {
          has_traffic_lights = true;
          traffic_light_passthrough.push_back(*matched_3d);
        } else {
          filtered_dets.detections.push_back(det2d);
          // Transform 3D detection position into LiDAR frame, then compute distance from LiDAR origin
          geometry_msgs::msg::Point pos_lidar;
          if (tf_det3d_to_lidar.has_value()) {
            geometry_msgs::msg::PointStamped ps_in, ps_out;
            ps_in.point = matched_3d->bbox.center.position;
            tf2::doTransform(ps_in, ps_out, tf_det3d_to_lidar.value());
            pos_lidar = ps_out.point;
          } else {
            pos_lidar = matched_3d->bbox.center.position;
          }
          const double depth =
            std::sqrt(pos_lidar.x * pos_lidar.x + pos_lidar.y * pos_lidar.y + pos_lidar.z * pos_lidar.z);
          std::optional<double> d;
          if (depth > min_depth_for_enrichment_) {
            d = depth;
          }
          cam_depths.push_back(d);
        }
      } else {
        // No matching 3D detection — keep 2D detection for association without depth
        if (cls2d != "traffic light") {
          filtered_dets.detections.push_back(det2d);
          cam_depths.push_back(std::nullopt);
        }
      }
    }

    // Use filtered detections (without traffic lights) when available, otherwise original
    const auto & dets_for_association = has_traffic_lights ? filtered_dets : camera_detections;

    auto detection_results = processDetections(
      dets_for_association,
      tf_lidar_to_cam,
      projection_matrix,
      cloud,
      std::move(candidates),
      lidar_header,
      cam_width,
      cam_height,
      cam_depths);

    const size_t nd = detection_results.detections3d.detections.size();
    if (nd == 0 && !camera_detections.detections.empty()) {
      ++cameras_with_2d_but_zero_3d;
    }
    const size_t ni = detection_results.lidar_indices_per_detection.size();
    const size_t nm = detection_results.bboxes.markers.size();
    if (ni != nd) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Camera %s: lidar_indices_per_detection (%zu) != 3D detections (%zu); cross-camera dedup overlap may be wrong",
        frame_id.c_str(),
        ni,
        nd);
    }
    for (size_t k = 0; k < nd; ++k) {
      CrossCameraWorkItem w;
      w.det3d = std::move(detection_results.detections3d.detections[k]);
      w.score = w.det3d.results.empty() ? 0.0 : static_cast<double>(w.det3d.results[0].hypothesis.score);
      if (k < ni) {
        w.sorted_indices = detection_results.lidar_indices_per_detection[k].indices;
        sortUniqueIndicesInPlace(w.sorted_indices);
      }
      w.has_bbox = publish_bounding_box_ && k < nm;
      if (w.has_bbox) {
        w.bbox = std::move(detection_results.bboxes.markers[k]);
        w.bbox.ns = frame_id;
      }
      accumulated.push_back(std::move(w));
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

  if (cross_camera_dedup_enabled_ && accumulated.size() > 1u) {
    const size_t before = accumulated.size();
    deduplicateCrossCameraWorkItems(
      accumulated,
      cross_camera_min_lidar_overlap_,
      cross_camera_weak_lidar_overlap_,
      cross_camera_max_center_dist_m_,
      cross_camera_min_bev_box_iou_,
      cross_camera_min_bev_box_iou_no_class_);
    if (debug_logging_ && before != accumulated.size()) {
      RCLCPP_INFO(get_logger(), "Cross-camera dedup: %zu -> %zu 3D detection(s)", before, accumulated.size());
    }
  }

  int bbox_id = 0;
  for (auto & w : accumulated) {
    merged_detections3d.detections.push_back(std::move(w.det3d));
    if (publish_bounding_box_ && w.has_bbox) {
      w.bbox.id = bbox_id++;
      merged_bboxes.markers.push_back(std::move(w.bbox));
    }
  }

  // Append traffic light detections that bypassed LiDAR association
  for (auto & tl : traffic_light_passthrough) {
    merged_detections3d.detections.push_back(std::move(tl));
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
    const size_t n_cams = msg->camera_detections.size();
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "No 3D detections despite %zu 2D (across %zu camera(s)). Skipped: stamp_delta=%zu no_camera_info=%zu "
      "tf_fail=%zu. Cameras that ran association but produced 0 3D: %zu. Hints: if "
      "stamp_delta==%zu increase max_detection_cloud_stamp_delta or clustered_cloud_cache_size; match detection "
      "frame_id to MultiCameraInfo; "
      "check TF %s→camera; relax object_detection_confidence, min_iou_threshold, or association_strict_matching.",
      total_2d,
      n_cams,
      skipped_stamp_delta,
      skipped_no_camera_info,
      skipped_tf_failed,
      cameras_with_2d_but_zero_3d,
      n_cams,
      lidar_frame_.c_str());
  }

  const int64_t pair_det_ns = rep_det_time.nanoseconds();
  const int64_t pair_cloud_ns = snap->cloud_stamp.nanoseconds();

  if (association_detection_seq != latest_detection_seq_.load(std::memory_order_acquire)) {
    return;
  }
  if (association_clustered_seq < latest_clustered_cloud_seq_.load(std::memory_order_acquire)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(publish_freshness_mutex_);
    const bool not_newer = (association_detection_seq < last_published_detection_seq_) ||
                           (association_detection_seq == last_published_detection_seq_ &&
                            association_clustered_seq <= last_published_clustered_seq_);
    if (not_newer) {
      if (
        skip_repeat_association_publish_ && last_association_publish_pair_valid_ &&
        pair_det_ns == last_association_publish_det_ns_ && pair_cloud_ns == last_association_publish_cloud_ns_)
      {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Skipping stale association publish (same timestamp pair det=%ld cloud=%ld)",
          pair_det_ns,
          pair_cloud_ns);
      }
      return;
    }
  }

  if (detection_3d_pub_ && detection_3d_pub_->is_activated()) {
    detection_3d_pub_->publish(merged_detections3d);
  }

  if (publish_bounding_box_ && bounding_box_pub_ && bounding_box_pub_->is_activated()) {
    visualization_msgs::msg::MarkerArray out_markers;
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = lidar_header.frame_id;
    delete_all.header.stamp = bbox_markers_use_detection_stamp_ ? timeToMsg(rep_det_time) : lidar_header.stamp;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    out_markers.markers.push_back(std::move(delete_all));

    if (bbox_markers_use_detection_stamp_) {
      const builtin_interfaces::msg::Time det_stamp_msg = timeToMsg(rep_det_time);
      for (auto & m : merged_bboxes.markers) {
        m.header.stamp = det_stamp_msg;
        m.header.frame_id = lidar_header.frame_id;
      }
    }
    for (auto & m : merged_bboxes.markers) {
      out_markers.markers.push_back(std::move(m));
    }
    bounding_box_pub_->publish(out_markers);
  }

  {
    std::lock_guard<std::mutex> lock(publish_freshness_mutex_);
    last_published_detection_seq_ = association_detection_seq;
    last_published_clustered_seq_ = association_clustered_seq;
    last_association_publish_pair_valid_ = true;
    last_association_publish_det_ns_ = pair_det_ns;
    last_association_publish_cloud_ns_ = pair_cloud_ns;
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
  int image_height,
  const std::vector<std::optional<double>> & detection_depths)
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
      "Camera %s: %zu clusters before association, %zu detections",
      transform.header.frame_id.c_str(),
      candidates.size(),
      detection.detections.size());
  }

  projection_utils::assignCandidatesToDetectionsByIOU(
    cloud,
    candidates,
    detection,
    transform,
    projection_matrix,
    object_detection_confidence_,
    image_width,
    image_height,
    detection_depths);

  if (debug_logging_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Camera %s: %zu clusters kept after association",
      transform.header.frame_id.c_str(),
      candidates.size());
  }

  if (candidates.empty()) {
    return detection_outputs;
  }

  projection_utils::filterCandidatesByClassAwareConstraints(candidates, detection);

  std::vector<pcl::PointIndices> out_indices = projection_utils::extractIndices(candidates);

  // Compute 3D boxes with class-aware fitting (L-shaped for vehicles).
  std::vector<projection_utils::Box3D> boxes;
  boxes.reserve(out_indices.size());
  for (size_t i = 0; i < out_indices.size(); ++i) {
    std::string class_hint;
    if (candidates[i].match && candidates[i].match->det_idx >= 0) {
      size_t di = static_cast<size_t>(candidates[i].match->det_idx);
      if (di < detection.detections.size() && !detection.detections[di].results.empty()) {
        class_hint = detection.detections[di].results[0].hypothesis.class_id;
      }
    }
    boxes.push_back(cluster_box::computeClusterBoxWithClassHint(cloud, out_indices[i], class_hint));
    cluster_box::applyClassAwareBoxExtension(boxes.back(), class_hint);
  }

  if (publish_bounding_box_) {
    detection_outputs.bboxes = projection_utils::computeBoundingBox(boxes, out_indices, lidar_header);
  }

  detection_outputs.detections3d = projection_utils::compute3DDetection(boxes, candidates, lidar_header, detection);

  detection_outputs.lidar_indices_per_detection.clear();
  detection_outputs.lidar_indices_per_detection.reserve(candidates.size());
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (candidates[i].indices.indices.empty()) continue;
    detection_outputs.lidar_indices_per_detection.push_back(candidates[i].indices);
  }

  if (publish_visualization_) {
    detection_outputs.colored_cluster.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    core_->assignClusterColors(cloud, out_indices, detection_outputs.colored_cluster);

    detection_outputs.centroid_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    core_->computeClusterCentroids(cloud, out_indices, detection_outputs.centroid_cloud);
  }
  return detection_outputs;
}

RCLCPP_COMPONENTS_REGISTER_NODE(SpatialAssociationNode)
