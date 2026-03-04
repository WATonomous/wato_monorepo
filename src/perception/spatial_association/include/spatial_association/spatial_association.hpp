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

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <camera_object_detection_msgs/msg/batch_detection.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

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

  // Topic names (remap in launch)
  static constexpr auto kCameraInfoFront = "camera_info_front";
  static constexpr auto kCameraInfoLeft = "camera_info_left";
  static constexpr auto kCameraInfoRight = "camera_info_right";
  static constexpr auto kNonGroundCloud = "non_ground_cloud";
  static constexpr auto kDetections = "detections";
  static constexpr auto kFilteredLidar = "filtered_lidar";
  static constexpr auto kClusterCentroid = "cluster_centroid";
  static constexpr auto kBoundingBox = "bounding_box";
  static constexpr auto kDetection3d = "detection_3d";

  /** Lifecycle: configure parameters and core (no subs/pubs). */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override;
  /** Lifecycle: create subs/pubs and activate publishers. */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;
  /** Lifecycle: deactivate publishers and stop processing. */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;
  /** Lifecycle: destroy subs/pubs and release core. */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &) override;
  /** Lifecycle: shutdown. */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  // CONFIG/VISUALIZATION
  bool publish_visualization_;
  bool debug_logging_;

  // CAMERA
  // ------------------------------------------------------------------------------------------------------

  std::unordered_map<std::string, sensor_msgs::msg::CameraInfo::SharedPtr> camInfoMap_;

  /**
   * @brief Callback for camera info messages from multiple cameras
   * @param msg Camera info message
   */
  void multiCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  /**
   * @brief Callback for non-ground filtered point cloud from patchwork
   * @param msg Non-ground point cloud message
   */
  void nonGroundCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  sensor_msgs::msg::PointCloud2 latest_lidar_msg_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud_;
  std::vector<pcl::PointIndices> cluster_indices;

  std::unique_ptr<SpatialAssociationCore> core_;

  /**
   * @brief Callback for batch detection messages from multiple cameras
   * @param msg Batch detection message containing detections from all cameras
   */
  void multiDetectionsCallback(camera_object_detection_msgs::msg::BatchDetection::SharedPtr msg);

  /**
   * @brief Processes 2D camera detections and matches them with 3D LiDAR clusters
   * @param detections 2D camera detections
   * @param transform Transform from LiDAR to camera frame
   * @param projection_matrix Camera projection matrix (3x4, flattened to 12 elements)
   * @return DetectionOutputs containing bounding boxes, detections, and visualization data
   * @note Uses member variable cluster_indices_ for processing
   */
  DetectionOutputs processDetections(
    const vision_msgs::msg::Detection2DArray & detections,
    const geometry_msgs::msg::TransformStamped & transform,
    const std::array<double, 12> & projection_matrix);

  geometry_msgs::msg::TransformStamped transform;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_cloud_sub_;
  rclcpp::Subscription<camera_object_detection_msgs::msg::BatchDetection>::SharedPtr batch_dets_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_front_, camera_info_sub_left_,
    camera_info_sub_right_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_lidar_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_centroid_pub_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_box_pub_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_3d_pub_;

  /**
   * @brief Initializes and retrieves ROS parameters
   */
  void initializeParams();

  std::string lidar_frame_;

  // Filtering parameters

  double euclid_cluster_tolerance_;
  int euclid_min_cluster_size_;
  int euclid_max_cluster_size_;
  bool use_adaptive_clustering_;
  double euclid_close_threshold_;
  double euclid_close_tolerance_mult_;

  double density_weight_;
  double size_weight_;
  double distance_weight_;
  double score_threshold_;

  double merge_threshold_;

  float object_detection_confidence_;

  // Voxel downsampling parameter
  float voxel_size_;

  // Working PCL objects for visualization (reused from core)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_colored_cluster_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr working_centroid_cloud_;
};

#endif
