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
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <deep_msgs/msg/multi_camera_info.hpp>
#include <deep_msgs/msg/multi_detection2_d_array.hpp>
#include <deep_msgs/msg/multi_image_compressed.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "spatial_association/spatial_association_core.hpp"
#include "utils/projection_utils.hpp"

struct DetectionOutputs
{
  visualization_msgs::msg::MarkerArray bboxes;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster;
  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud;
  vision_msgs::msg::Detection3DArray detections3d;
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
  void multiDetectionCallback(const deep_msgs::msg::MultiDetection2DArray::SharedPtr msg);

  DetectionOutputs processDetections(
    const vision_msgs::msg::Detection2DArray & detections,
    const geometry_msgs::msg::TransformStamped & transform,
    const std::array<double, 12> & projection_matrix,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    std::vector<projection_utils::ClusterCandidate> candidates,
    const std_msgs::msg::Header & lidar_header,
    int image_width = 0,
    int image_height = 0);

  std::mutex cloud_mutex_;
  std_msgs::msg::Header latest_lidar_header_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud_;
  std::vector<pcl::PointIndices> cluster_indices;

  std::mutex candidates_cache_mutex_;
  rclcpp::Time cached_candidates_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<projection_utils::ClusterCandidate> cached_candidates_;

  std::thread worker_thread_;
  std::mutex job_mutex_;
  std::condition_variable job_cv_;
  std::atomic<bool> worker_shutdown_{false};
  std::atomic<uint64_t> cloud_job_id_{0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr pending_cloud_;
  std_msgs::msg::Header pending_header_;
  uint64_t pending_job_id_{0};  // 0 means no pending job

  std::unique_ptr<SpatialAssociationCore> core_;

  std::mutex transform_cache_mutex_;
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
  double max_detection_cloud_stamp_delta_{0.0};

  double euclid_cluster_tolerance_;
  int euclid_min_cluster_size_;
  int euclid_max_cluster_size_;
  bool use_adaptive_clustering_;
  double euclid_close_threshold_;
  double euclid_close_tolerance_mult_;

  double merge_threshold_;

  float object_detection_confidence_;

  float voxel_size_;
};

#endif
