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

#ifndef PROJECTION_UTILS_HPP
#define PROJECTION_UTILS_HPP

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <optional>
#include <random>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ProjectionUtils
{
public:
  struct Box3D
  {
    Eigen::Vector3f center{0.f, 0.f, 0.f};
    Eigen::Vector3f size{0.f, 0.f, 0.f};  // length (x), width (y), height (z)
    double yaw{0.0};
  };

  struct ClusterStats
  {
    Eigen::Vector4f centroid;  // x,y,z,1
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
    int num_points;
  };

  /**
   * @brief Projects a 3D LiDAR point onto a 2D camera image plane
   * @param transform Transform from LiDAR frame to camera frame
   * @param p Camera projection matrix (3x4, flattened to 12 elements)
   * @param pt 3D point in LiDAR frame
   * @return 2D image coordinates if projection is valid and within image bounds, nullopt otherwise
   */
  static std::optional<cv::Point2d> projectLidarToCamera(
    const geometry_msgs::msg::TransformStamped & transform, const std::array<double, 12> & p, const pcl::PointXYZ & pt);

  /**
   * @brief Performs Euclidean clustering on a point cloud
   * @param cloud Input point cloud (modified in place)
   * @param clusterTolerance Maximum distance between points in the same cluster
   * @param minClusterSize Minimum number of points required for a valid cluster
   * @param maxClusterSize Maximum number of points allowed in a cluster
   * @param cluster_indices Output vector of cluster indices
   */
  static void euclideanClusterExtraction(
    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    double clusterTolerance,
    int minClusterSize,
    int maxClusterSize,
    std::vector<pcl::PointIndices> & cluster_indices);

  /**
   * @brief Performs adaptive Euclidean clustering with distance-based tolerance
   * @param cloud Input point cloud (modified in place)
   * @param base_cluster_tolerance Base cluster tolerance for far points
   * @param minClusterSize Minimum number of points required for a valid cluster
   * @param maxClusterSize Maximum number of points allowed in a cluster
   * @param cluster_indices Output vector of cluster indices
   * @param close_threshold Distance threshold to classify points as "close"
   * @param close_tolerance_mult Multiplier for cluster tolerance on close points
   * @details Uses larger cluster tolerance for closer points to prevent fragmentation
   */
  static void adaptiveEuclideanClusterExtraction(
    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    double base_cluster_tolerance,
    int minClusterSize,
    int maxClusterSize,
    std::vector<pcl::PointIndices> & cluster_indices,
    double close_threshold = 10.0,
    double close_tolerance_mult = 1.5);

  /**
   * @brief Assigns random colors to clusters for visualization
   * @param cloud Input point cloud
   * @param cluster_indices Vector of cluster indices
   * @param clustered_cloud Output colored point cloud
   */
  static void assignClusterColors(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::vector<pcl::PointIndices> & cluster_indices,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & clustered_cloud);

  /**
   * @brief Computes statistics for each cluster (centroid, bounds, point count)
   * @param cloud Input point cloud
   * @param cluster_indices Vector of cluster indices
   * @return Vector of cluster statistics
   */
  static std::vector<ClusterStats> computeClusterStats(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices);

  /**
   * @brief Merges clusters using precomputed statistics
   * @param cluster_indices Vector of cluster indices (modified in place)
   * @param cloud Input point cloud
   * @param stats Precomputed cluster statistics
   * @param mergeTolerance Maximum distance between centroids for merging
   */
  static void mergeClusters(
    std::vector<pcl::PointIndices> & cluster_indices,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::vector<ClusterStats> & stats,
    double mergeTolerance);

  /**
   * @brief Multi-stage cluster filtering with physics-based constraints
   * @param stats Precomputed cluster statistics
   * @param cluster_indices Vector of cluster indices (modified in place)
   * @param max_distance Maximum distance from sensor to keep cluster
   * @param enable_debug Enable debug output of filtering statistics
   */
  static void filterClusterByQuality(
    const std::vector<ClusterStats> & stats,
    std::vector<pcl::PointIndices> & cluster_indices,
    double max_distance = 60.0,
    bool enable_debug = false);

  /**
   * @brief Filters clusters using physics-based constraints with distance-adaptive thresholds
   * @param stats Precomputed cluster statistics
   * @param cluster_indices Vector of cluster indices (modified in place)
   * @param max_distance Maximum distance from sensor to keep cluster
   * @param min_points Minimum points for viable object
   * @param min_height Minimum height in meters
   * @param min_points_default Default minimum points
   * @param min_points_far Minimum points at far distance
   * @param min_points_medium Minimum points at medium distance
   * @param min_points_large Minimum points for large objects
   * @param distance_threshold_far Distance threshold for far objects (m)
   * @param distance_threshold_medium Distance threshold for medium objects (m)
   * @param volume_threshold_large Volume threshold for large objects (m³)
   * @param min_density Minimum point density (points per m³)
   * @param max_density Maximum point density (points per m³)
   * @param max_dimension Maximum object dimension in meters
   * @param max_aspect_ratio Maximum aspect ratio
   */
  static void filterClustersByPhysicsConstraints(
    const std::vector<ClusterStats> & stats,
    std::vector<pcl::PointIndices> & cluster_indices,
    double max_distance,
    int min_points,
    float min_height,
    int min_points_default,
    int min_points_far,
    int min_points_medium,
    int min_points_large,
    double distance_threshold_far,
    double distance_threshold_medium,
    float volume_threshold_large,
    float min_density,
    float max_density,
    float max_dimension,
    float max_aspect_ratio);

  /**
   * @brief Computes the centroid of a cluster
   * @param cloud Input point cloud
   * @param cluster_indices Indices of points in the cluster
   * @param centroid Output centroid point
   * @return true if centroid was computed successfully, false otherwise
   */
  static bool computeClusterCentroid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const pcl::PointIndices & cluster_indices,
    pcl::PointXYZ & centroid);

  /**
   * @brief Computes maximum IoU using precomputed cluster statistics
   * @param cluster_stats Precomputed cluster statistics
   * @param transform Transform from LiDAR to camera frame
   * @param projection_matrix Camera projection matrix (3x4, flattened to 12 elements)
   * @param detections 2D camera detections
   * @param object_detection_confidence Minimum confidence threshold for detections
   * @return Maximum IoU score between cluster projection and detections
   */
  static double computeMaxIOU8Corners(
    const ClusterStats & cluster_stats,
    const geometry_msgs::msg::TransformStamped & transform,
    const std::array<double, 12> & projection_matrix,
    const vision_msgs::msg::Detection2DArray & detections,
    const float object_detection_confidence);

  /**
   * @brief Filters clusters using precomputed statistics to keep only the best IoU match
   * @param stats Precomputed cluster statistics
   * @param cluster_indices Vector of cluster indices (modified in place)
   * @param detections 2D camera detections
   * @param transform Transform from LiDAR to camera frame
   * @param projection_matrix Camera projection matrix (3x4, flattened to 12 elements)
   * @param object_detection_confidence Minimum confidence threshold for detections
   */
  static void computeHighestIOUCluster(
    const std::vector<ClusterStats> & stats,
    std::vector<pcl::PointIndices> & cluster_indices,
    const vision_msgs::msg::Detection2DArray & detections,
    const geometry_msgs::msg::TransformStamped & transform,
    const std::array<double, 12> & projection_matrix,
    const float object_detection_confidence);

  /**
   * @brief Filters out ground noise clusters
   * @param stats Precomputed cluster statistics
   * @param cluster_indices Vector of cluster indices (modified in place)
   */
  static void filterGroundNoise(
    const std::vector<ClusterStats> & stats, std::vector<pcl::PointIndices> & cluster_indices);

  /**
   * @brief Computes 3D bounding boxes for all clusters
   * @param cloud Input point cloud
   * @param cluster_indices Vector of cluster indices
   * @return Vector of 3D bounding boxes
   */
  static std::vector<Box3D> computeClusterBoxes(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices);

  /**
     * @brief Converts precomputed boxes to visualization marker array
     * @param boxes Precomputed 3D bounding boxes
     * @param cluster_indices Vector of cluster indices
     * @param msg Point cloud message for header information
     * @return Marker array for visualization
     */
  static visualization_msgs::msg::MarkerArray computeBoundingBox(
    const std::vector<Box3D> & boxes,
    const std::vector<pcl::PointIndices> & cluster_indices,
    const sensor_msgs::msg::PointCloud2 & msg);

  /**
     * @brief Converts precomputed boxes to 3D detection array
     * @param boxes Precomputed 3D bounding boxes
     * @param cluster_indices Vector of cluster indices
     * @param msg Point cloud message for header information
     * @return Detection3DArray message
     */
  static vision_msgs::msg::Detection3DArray compute3DDetection(
    const std::vector<Box3D> & boxes,
    const std::vector<pcl::PointIndices> & cluster_indices,
    const sensor_msgs::msg::PointCloud2 & msg);

private:
  static const int image_width_ = 1600;
  static const int image_height_ = 900;
};

#endif
