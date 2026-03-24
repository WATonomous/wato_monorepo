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

#ifndef SPATIAL_ASSOCIATION_HPP
#define SPATIAL_ASSOCIATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <unordered_map>
#include <vector>

#include <deep_msgs/msg/multi_camera_info.hpp>
#include <deep_msgs/msg/multi_detection2_d_array.hpp>
#include <deep_msgs/msg/multi_image_compressed.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "spatial_association/spatial_association_core.hpp"
#include "utils/projection_utils.hpp"

struct DetectionOutputs
{
  visualization_msgs::msg::MarkerArray bboxes;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster;
  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud;
  vision_msgs::msg::Detection3DArray detections3d;
  /** Same length and order as @c detections3d.detections (LiDAR voxel indices into the filtered cloud). */
  std::vector<pcl::PointIndices> lidar_indices_per_detection;
};

class SpatialAssociationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  SpatialAssociationNode() = delete;
  explicit SpatialAssociationNode(const rclcpp::NodeOptions & options);
  ~SpatialAssociationNode() override;

  static constexpr auto kMultiCameraInfo = "multi_camera_info";
  static constexpr auto kMultiImage = "multi_image";
  static constexpr auto kNonGroundCloud = "non_ground_cloud";
  static constexpr auto kDetections = "detections";
  static constexpr auto kFilteredLidar = "filtered_lidar";
  static constexpr auto kClusterCentroid = "cluster_centroid";
  static constexpr auto kBoundingBox = "bounding_box";
  static constexpr auto kDetection3d = "detection_3d";

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  struct ClusteredCloudSnapshot;

  bool publish_bounding_box_;
  bool publish_visualization_;
  bool publish_image_visualization_;
  bool debug_logging_;

  std::mutex camera_info_mutex_;
  deep_msgs::msg::MultiCameraInfo::SharedPtr latest_multi_camera_info_;
  std::unordered_map<std::string, size_t> frame_id_to_index_;

  std::mutex image_mutex_;
  deep_msgs::msg::MultiImageCompressed::SharedPtr latest_multi_image_;

  void multiCameraInfoCallback(const deep_msgs::msg::MultiCameraInfo::SharedPtr msg);
  void multiImageCallback(const deep_msgs::msg::MultiImageCompressed::SharedPtr msg);
  void nonGroundCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void workerLoop();
  void stopWorker();
  void associationLoop();
  void stopAssociationWorker();
  void notifyAssociationWorker(std::shared_ptr<const ClusteredCloudSnapshot> forced_snapshot_hint = nullptr);
  void multiDetectionCallback(const deep_msgs::msg::MultiDetection2DArray::SharedPtr msg);
  /** Core association + publish; serialized by @c association_mutex_. Forced snapshot is treated as a hint. */
  void runAssociationFromDetections(
    const deep_msgs::msg::MultiDetection2DArray::SharedPtr msg,
    std::shared_ptr<const ClusteredCloudSnapshot> forced_snapshot = nullptr);

  /** With @c cloud_mutex_ held: bump invalidation generation and clear @c clustered_cloud_cache_. */
  void invalidateClusteredCloudCacheLocked();
  /** After @c stopWorker(): clear clustered cache (with bump), TF cache; shared by cleanup/shutdown. */
  void resetCachesAfterWorkerStopped();

  DetectionOutputs processDetections(
    const vision_msgs::msg::Detection2DArray & detections,
    const geometry_msgs::msg::TransformStamped & transform,
    const std::array<double, 12> & projection_matrix,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    std::vector<projection_utils::ClusterCandidate> candidates,
    const std_msgs::msg::Header & lidar_header,
    int image_width = 0,
    int image_height = 0,
    bool skip_iou_assignment = false,
    bool apply_pre_assoc_constraints = true,
    bool use_tiered_post_assoc_constraints = true);

  /** Filtered + clustered LiDAR for one non-ground message; shared into @c clustered_cloud_cache_. */
  struct ClusteredCloudSnapshot
  {
    uint64_t seq{0};
    std_msgs::msg::Header header;
    /** Same instant as @c header.stamp; avoids repeated rclcpp::Time construction. */
    rclcpp::Time cloud_stamp{0, 0, RCL_ROS_TIME};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<pcl::PointIndices> cluster_indices;
    /** Built in worker when @c use_roi_first_clustering_ is false; unused otherwise. */
    std::optional<std::vector<projection_utils::ClusterCandidate>> global_candidates;
  };

  struct PendingRawCloudJob
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std_msgs::msg::Header header;
    uint64_t seq{0};
  };

  std::mutex cloud_mutex_;
  std::atomic<uint64_t> clustered_cache_invalidation_generation_{0};
  std::deque<std::shared_ptr<ClusteredCloudSnapshot>> clustered_cloud_cache_;
  size_t clustered_cloud_cache_size_{24};
  /** Call with @c cloud_mutex_ held. Closest cached snapshot to @p det_time, honoring @c max_detection_cloud_stamp_delta_. */
  std::shared_ptr<ClusteredCloudSnapshot> pickNearestClusteredCloudLocked(const rclcpp::Time & det_time);

  std::thread worker_thread_;
  std::mutex job_mutex_;
  std::condition_variable job_cv_;
  std::atomic<bool> worker_shutdown_{false};
  std::atomic<uint64_t> cloud_job_id_{0};
  std::atomic<uint64_t> latest_detection_seq_{0};
  std::atomic<uint64_t> latest_raw_cloud_seq_{0};
  std::atomic<uint64_t> latest_clustered_cloud_seq_{0};
  std::deque<PendingRawCloudJob> pending_cloud_queue_;
  /** Max raw clouds queued when @c raw_cloud_latest_only_ is false; excess drops oldest. */
  size_t pending_cloud_queue_max_{1};
  /** If true, each new cloud replaces the entire pending queue (freshest LiDAR for the worker). */
  bool raw_cloud_latest_only_{true};

  std::mutex latest_detections_mutex_;
  deep_msgs::msg::MultiDetection2DArray::SharedPtr latest_detections_;
  std::mutex association_mutex_;
  std::thread association_thread_;
  std::mutex association_trigger_mutex_;
  std::condition_variable association_trigger_cv_;
  bool association_trigger_pending_{false};
  bool association_shutdown_{false};
  std::shared_ptr<const ClusteredCloudSnapshot> association_forced_snapshot_hint_;

  std::unique_ptr<SpatialAssociationCore> core_;

  std::mutex transform_cache_mutex_;
  /** Static lidar→camera extrinsics: looked up with tf2::TimePointZero only; restart node if calibration changes. */
  std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> lidar_to_cam_transform_cache_;

  rclcpp::Subscription<deep_msgs::msg::MultiCameraInfo>::SharedPtr multi_camera_info_sub_;
  rclcpp::Subscription<deep_msgs::msg::MultiImageCompressed>::SharedPtr multi_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_cloud_sub_;
  rclcpp::Subscription<deep_msgs::msg::MultiDetection2DArray>::SharedPtr dets_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_lidar_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_centroid_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_box_pub_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_3d_pub_;

  void initializeParams();

  std::string lidar_frame_;
  /** Max |detection time − cloud time| (s); ≤0 disables rejection (still pick nearest cached cloud). */
  double max_detection_cloud_stamp_delta_{0.08};
  /** Fallback for zero MultiDetection header stamp: median_camera_stamp (default) or max_camera_stamp. */
  std::string detection_stamp_fallback_policy_{"median_camera_stamp"};

  double euclid_cluster_tolerance_;
  int euclid_min_cluster_size_;
  int euclid_roi_min_cluster_size_;
  int euclid_max_cluster_size_;
  bool use_adaptive_clustering_;
  double euclid_close_threshold_;
  double euclid_mid_threshold_;
  double euclid_band_near_mid_overlap_m_;
  double euclid_band_mid_far_overlap_m_;
  double euclid_band_merge_min_index_overlap_;
  double euclid_near_tolerance_mult_;
  double euclid_mid_tolerance_mult_;
  double euclid_far_tolerance_mult_;

  double merge_threshold_;

  float object_detection_confidence_;

  float voxel_size_;

  /** When true, cluster only points projecting into each 2D detection ROI (skips global cluster association). */
  bool use_roi_first_clustering_{false};
  /** When true and ROI-first is enabled, skip voxel downsampling in core pre-processing. */
  bool skip_voxel_for_roi_first_{false};

  /** RViz-only: stamp bbox markers with batch detection time; does not fix real LiDAR–det alignment (see @c skip_repeat_association_publish_). */
  bool bbox_markers_use_detection_stamp_{false};
  /** If true, emit debug logs for repeated timestamp-pair associations (pair is no longer used for suppression). */
  bool skip_repeat_association_publish_{true};

  std::mutex publish_freshness_mutex_;
  uint64_t last_published_detection_seq_{0};
  uint64_t last_published_clustered_seq_{0};
  bool last_association_publish_pair_valid_{false};
  int64_t last_association_publish_det_ns_{0};
  int64_t last_association_publish_cloud_ns_{0};
  std::vector<std::pair<std::string, int32_t>> last_published_bbox_marker_keys_;

  /** Same as quality_filter_params.max_distance; applied as core max lidar range pre-voxel. */
  double quality_max_distance_{60.0};

  bool cross_camera_dedup_enabled_{true};
  /** intersection / min(|A|,|B|) on sorted LiDAR indices — same cluster in non-ROI mode often ≈1. */
  double cross_camera_min_lidar_overlap_{0.5};
  /** With same class_id, merge if overlap ≥ this and BEV center distance ≤ @c cross_camera_max_center_dist_m. */
  double cross_camera_weak_lidar_overlap_{0.22};
  double cross_camera_max_center_dist_m_{1.8};
  /** BEV axis-aligned IoU of oriented box footprints; merges same object seen by different cameras with disjoint LiDAR indices. */
  double cross_camera_min_bev_box_iou_{0.16};
  /** Stricter BEV IoU when both hypotheses lack class_id. */
  double cross_camera_min_bev_box_iou_no_class_{0.30};
  /** Minimum Z-overlap ratio (intersection / min(height_a,height_b)) required for BEV-IoU dedup paths. */
  double cross_camera_min_z_overlap_ratio_{0.20};
};

#endif
