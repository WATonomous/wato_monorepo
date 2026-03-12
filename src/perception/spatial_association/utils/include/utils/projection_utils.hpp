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

#include <Eigen/Dense>
#include <array>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>
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

  /** One cluster–detection pair from IoU matching (cluster_idx, det_idx, iou). */
  struct ClusterDetectionMatch
  {
    size_t cluster_idx{0};
    int det_idx{-1};
    double iou{0.0};
  };

  /**
   * Single container for a cluster through the pipeline: indices, stats, and optional 2D match.
   * Replaces parallel vectors (cluster_indices, stats, matches) so alignment cannot go stale.
   */
  struct ClusterCandidate
  {
    pcl::PointIndices indices;
    ClusterStats stats;
    std::optional<ClusterDetectionMatch> match;
  };

  /**
   * @brief Build candidates from cluster indices (computes stats, match = nullopt).
   * @param cloud Input point cloud
   * @param cluster_indices Vector of cluster indices (from clustering + merge)
   * @return Vector of ClusterCandidate with indices and stats filled
   */
  static std::vector<ClusterCandidate> buildCandidates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::vector<pcl::PointIndices> & cluster_indices);

  /**
   * @brief Extract cluster indices from candidates (for APIs that still take indices).
   */
  static std::vector<pcl::PointIndices> extractIndices(const std::vector<ClusterCandidate> & candidates);

  /**
   * @brief Configurable parameters for projection/utils (orientation, filtering, visualization).
   * Set via setParams(); defaults match previous hardcoded constants in projection_utils.cpp.
   */
  struct ProjectionUtilsParams
  {
    // Visualization
    double marker_lifetime_s = 0.5;
    float marker_alpha = 0.2f;

    // IoU matching (cluster <-> 2D detection)
    double min_iou_threshold = 0.15;
    // 3D detection score = detection_score_weight * det_score + iou_score_weight * iou
    double detection_score_weight = 0.6;
    double iou_score_weight = 0.4;

    // Bounding box orientation
    double ar_front_view_threshold = 1.2;

    // Outlier rejection
    size_t outlier_rejection_point_count = 30;
    double outlier_sigma_multiplier = 4.5;

    // Trimmed extents (percentiles 0–100) for robust box fitting; reduces impact of stray points and ground leak
    double xy_extent_percentile_low = 5.0;
    double xy_extent_percentile_high = 95.0;
    double z_extent_percentile_low = 2.0;
    double z_extent_percentile_high = 98.0;

    // Orientation search (coarse/fine derived: coarse = 5*step, fine_range = 2.5*step)
    size_t min_points_for_fit = 3;
    size_t default_sample_point_count = 64;
    double orientation_search_step_degrees = 2.0;

    // Camera projection
    double min_camera_z_distance = 1.0;
  };

  /** Set global params (used by all static methods). Call from node after reading ROS params. */
  static void setParams(const ProjectionUtilsParams & params);
  /** Get current params. */
  static const ProjectionUtilsParams & getParams();

  /**
   * @brief Build lidar-to-image matrix once for use in point loops.
   * Use this + projectLidarToCamera(lidar_to_image, pt) so matrices are not rebuilt per point.
   */
  static Eigen::Matrix<double, 3, 4> buildLidarToImageMatrix(
    const geometry_msgs::msg::TransformStamped & transform,
    const std::array<double, 12> & projection_matrix);

  /**
   * @brief Projects a 3D LiDAR point using a precomputed lidar-to-image matrix (P * T).
   * For point loops: build the matrix once with buildLidarToImageMatrix(), then call this per point.
   * @param lidar_to_image Combined 3x4 matrix (camera projection * camera extrinsic)
   * @param pt 3D point in LiDAR frame
   * @return 2D image coordinates if projection is valid (z > min_camera_z_distance); no image-bound check (caller clips if needed).
   */
  static std::optional<cv::Point2d> projectLidarToCamera(
    const Eigen::Matrix<double, 3, 4> & lidar_to_image, const pcl::PointXYZ & pt);

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
   * @brief Physics filter on candidates (uses candidate.stats; removes failed in place).
   */
  static void filterCandidatesByPhysicsConstraints(
    std::vector<ClusterCandidate> & candidates,
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
   * @brief Class-aware filter on candidates (uses candidate.stats and candidate.match; removes failed in place).
   */
  static void filterCandidatesByClassAwareConstraints(
    std::vector<ClusterCandidate> & candidates,
    const vision_msgs::msg::Detection2DArray & detections,
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
    float default_max_dimension,
    float default_max_aspect_ratio);

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
   * @brief Greedy one-to-one assignment by IoU using 8-corner AABB projection from candidate stats.
   * Fills candidate.match for kept, removes unmatched (in place).
   */
  static void assignCandidatesToDetectionsByIOU(
    std::vector<ClusterCandidate> & candidates,
    const vision_msgs::msg::Detection2DArray & detections,
    const geometry_msgs::msg::TransformStamped & transform,
    const std::array<double, 12> & projection_matrix,
    float object_detection_confidence);

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
     * @param header Header (frame_id, stamp) for output messages
     * @return Marker array for visualization
     */
  static visualization_msgs::msg::MarkerArray computeBoundingBox(
    const std::vector<Box3D> & boxes,
    const std::vector<pcl::PointIndices> & cluster_indices,
    const std_msgs::msg::Header & header);

  /**
   * @brief 3D detection array from boxes and candidates (reads class/score from candidate.match).
   */
  static vision_msgs::msg::Detection3DArray compute3DDetection(
    const std::vector<Box3D> & boxes,
    const std::vector<ClusterCandidate> & candidates,
    const std_msgs::msg::Header & header,
    const vision_msgs::msg::Detection2DArray & detections);

private:
  static const int image_width_ = 1280;
  static const int image_height_ = 1024;
  static ProjectionUtilsParams s_params_;
};

#endif
