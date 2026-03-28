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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/**
 * @brief Static utilities for LiDAR-camera projection, clustering, merging, IoU assignment,
 *        3D box fitting, and post-IoU candidate quality filtering.
 */
namespace projection_utils
{

/** 3D oriented bounding box: center, size (length-x, width-y, height-z in box frame), and yaw (rad). */
struct Box3D
{
  Eigen::Vector3f center{0.f, 0.f, 0.f};
  Eigen::Vector3f size{0.f, 0.f, 0.f}; /**< length (x), width (y), height (z) in box frame */
  double yaw{0.0};
};

/** Per-cluster statistics: centroid (x,y,z,1), axis-aligned bounds, and point count. */
struct ClusterStats
{
  Eigen::Vector4f centroid;
  float min_x, max_x;
  float min_y, max_y;
  float min_z, max_z;
  int num_points;
};

/** One cluster-detection pair from IoU matching. */
struct ClusterDetectionMatch
{
  size_t cluster_idx{0};
  int det_idx{-1};
  double iou{0.0};
};

/**
 * Single container for a cluster through the pipeline: indices, stats, and optional 2D match.
 * Keeps indices, stats, and match aligned (replaces parallel vectors).
 */
struct ClusterCandidate
{
  pcl::PointIndices indices;
  ClusterStats stats;
  std::optional<ClusterDetectionMatch> match;
};

/**
 * @brief Single struct for all projection_utils tunables (process-global via @ref setParams).
 *
 * Typical layout when driven from spatial_association:
 * - ROS @c projection_utils_params.* -> markers, IoU, second pass, image size, box-fit percentiles, etc.
 * - ROS @c quality_filter_params.* -> @c quality_* fields for post-IoU retention
 *   (@ref filterCandidatesByClassAwareConstraints)
 *
 * @note Defaults match former in-source constants. Call @ref setParams after reading ROS params
 *       so every entry point sees a consistent snapshot.
 */
struct ProjectionUtilsParams
{
  double marker_lifetime_s = 0.5;
  float marker_alpha = 0.2f;

  double min_iou_threshold = 0.15;
  double detection_score_weight = 0.6;
  double iou_score_weight = 0.4;

  double ar_front_view_threshold = 1.2;

  size_t outlier_rejection_point_count = 30;
  double outlier_sigma_multiplier = 4.5;

  double xy_extent_percentile_low = 5.0;
  double xy_extent_percentile_high = 95.0;
  double z_extent_percentile_low = 2.0;
  double z_extent_percentile_high = 98.0;

  size_t min_points_for_fit = 3;
  size_t default_sample_point_count = 64;
  double orientation_search_step_degrees = 2.0;

  double min_camera_z_distance = 1.0;

  int image_width = 0;
  int image_height = 0;

  bool enable_second_pass_fallback = false;
  double second_pass_min_iou = 0.05;
  int max_unassigned_detections_second_pass = 10;

  // Second-pass rescue policy (stronger evidence gates than IoU-only).
  // These are tuned to preserve recall while reducing false associations.
  double second_pass_min_det_conf = -1.0;  // If < 0, uses `object_detection_confidence` only.
  double second_pass_bbox_expand_fraction = 0.15;  // Expands det width/height by (1 + fraction).

  // Early rejection of very small 2D boxes (px^2). Set <= 0 to disable.
  double second_pass_min_det_area_px2 = 0.0;
  double second_pass_min_det_area_far_px2 = 0.0;

  // Support checks: projected points that fall inside the expanded detection bbox.
  // Near/medium range: require both count and fraction. Far range: require either (sparse points).
  int second_pass_min_inside_points = 3;
  double second_pass_min_inside_fraction = 0.20;
  double second_pass_inside_points_far_scale = 0.75;
  double second_pass_inside_points_medium_scale = 0.90;

  // Class-aware tightening (string class_id comes from det.results[0].hypothesis.class_id).
  // Expected sets: "person"; and vehicles: "car", "truck", "bus".
  double second_pass_person_inside_fraction_scale = 1.20;
  double second_pass_person_inside_points_scale = 1.20;
  double second_pass_vehicle_inside_fraction_scale = 0.90;
  double second_pass_vehicle_inside_points_scale = 0.90;

  // Dimension/size plausibility via projected footprint match (ratio score in [0,1]).
  double second_pass_min_size_score = 0.40;
  double second_pass_person_min_size_score = 0.50;
  double second_pass_vehicle_min_size_score = 0.35;

  // Combined-score acceptance: best score must be high and clearly above 2nd best.
  // Combined score uses: 0.35*iou + 0.30*inside_fraction + 0.20*center_score + 0.15*size_score.
  double second_pass_min_combined_score = 0.45;
  double second_pass_best_second_margin = 0.10;

  /** @{ @name Quality filter (post-IoU)
   *  Retention rules on @ref ClusterCandidate::stats : global min points/height, distance- and
   *  volume-dependent point floors, optional density check, max AABB extent and aspect ratio,
   *  max centroid range from origin. ROS: @c quality_filter_params.* in spatial_association.
   */
  /** Max distance from lidar origin (m); clusters beyond are dropped. */
  double quality_max_distance = 60.0;
  /** Minimum points and AABB height (m) for any candidate. */
  int quality_min_points = 5;
  float quality_min_height = 0.15f;
  /** Baseline min points before distance/volume tiering. */
  int quality_min_points_default = 10;
  /** Min points when centroid distance exceeds @ref quality_distance_threshold_far. */
  int quality_min_points_far = 8;
  /** Min points when distance exceeds @ref quality_distance_threshold_medium (but not far). */
  int quality_min_points_medium = 12;
  /** Min points when AABB volume exceeds @ref quality_volume_threshold_large (near range). */
  int quality_min_points_large = 30;
  double quality_distance_threshold_far = 30.0;
  double quality_distance_threshold_medium = 20.0;
  float quality_volume_threshold_large = 8.0f;
  /** Allowed points-per-unit-volume band when volume is non-trivial. */
  float quality_min_density = 5.0f;
  float quality_max_density = 1000.0f;
  /** Max largest AABB edge length (m). */
  float quality_max_dimension = 15.0f;
  /** Max ratio of largest to smallest AABB edge (thin slivers rejected). */
  float quality_max_aspect_ratio = 15.0f;
  /** @} */

  /** @{ @name Strict association (primary IoU pass)
   *  When @c association_strict_matching is true, a candidate pairs with a detection only if all
   *  gates pass (inner-box centroid, IoU on projected point hull or AABB, in-box point fraction,
   *  tiered min points, 2D aspect consistency). Disables permissive centroid-only fallback unless
   *  @c association_allow_aabb_centroid_fallback is true.
   */
  bool association_strict_matching = true;
  /** Centroid must lie inside a box this fraction of the 2D detection width/height (centered). 0.75 ≈ inner 75%. */
  double association_centroid_inner_box_fraction = 0.75;
  /** Min fraction of cluster points that project inside the (unexpanded) 2D detection box. */
  double association_min_inside_point_fraction = 0.45;
  /** min(projected AR / det AR, det AR / projected AR); below threshold rejects the pair. */
  double association_min_ar_consistency_score = 0.30;
  /** If true, IoU uses the image-axis bounding rect of projected cluster points (tighter than 3D AABB corners). */
  bool association_use_point_projection_rect_for_iou = true;
  /** Legacy: allow centroid-in-box with synthetic IoU when 8-corner AABB projection fails. */
  bool association_allow_aabb_centroid_fallback = false;
  /** If true (default), second-pass rescue is skipped while strict matching is on. */
  bool association_suppress_second_pass_under_strict = true;

  /** Weight of the depth-proximity term when enriched 3D detections are available. */
  double depth_score_weight = 0.15;
  /** Scale (metres) for depth score: exp(-|delta| / scale). */
  double depth_score_scale = 5.0;

  /** Expand each 2D box by this fraction (of width/height) when gathering LiDAR for ROI-first clustering. */
  double association_roi_expand_fraction = 0.12;
  /** Class-tight 3D caps after match (meters); applied in @ref filterCandidatesByClassAwareConstraints. */
  float quality_person_max_height_m = 2.5f;
  float quality_person_max_footprint_xy_m = 1.45f;
  float quality_vehicle_max_height_m = 4.8f;
  /** @} */
};

constexpr int kDefaultImageWidth = 1280;
constexpr int kDefaultImageHeight = 1024;

/**
 * @brief Installs @p params as the process-global snapshot for all functions.
 * @param params Values from ROS (spatial_association loads @c projection_utils_params.* and
 *               @c quality_filter_params.* into one struct).
 */
void setParams(const ProjectionUtilsParams & params);
/** @brief Reference to the last @ref setParams payload (read by IoU, box fit, quality filter, etc.). */
const ProjectionUtilsParams & getParams();

/**
 * @brief Build candidates from cluster indices (computes stats, match = nullopt).
 * @param cloud Input point cloud
 * @param cluster_indices Vector of cluster indices (from clustering + merge)
 * @return Vector of ClusterCandidate with indices and stats filled
 */
std::vector<ClusterCandidate> buildCandidates(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices);

/**
 * @brief For each 2D detection (high score first), Euclidean-cluster LiDAR points that project into an
 *        expanded image ROI; keep the best cluster and pre-fill @c candidate.match.
 *
 * Intended as ROI-first alternative to clustering the full cloud then associating.
 *
 * @param claim_points_unique If true, a point is consumed by the first (highest-score) detection that uses it.
 */
std::vector<ClusterCandidate> buildCandidatesFromDetectionRois(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const Eigen::Matrix<double, 3, 4> & lidar_to_image,
  const vision_msgs::msg::Detection2DArray & detections,
  float object_detection_confidence,
  double roi_expand_fraction,
  double cluster_tolerance,
  int min_cluster_size,
  int max_cluster_size,
  int image_width,
  int image_height,
  bool claim_points_unique);

/**
 * @brief Extracts cluster indices from candidates (for APIs that take indices).
 * @param candidates ClusterCandidate vector (order preserved)
 * @return Vector of point indices, one per candidate
 */
std::vector<pcl::PointIndices> extractIndices(const std::vector<ClusterCandidate> & candidates);

/**
 * @brief Build lidar-to-image matrix once for use in point loops.
 * Use this + projectLidarToCamera(lidar_to_image, pt) so matrices are not rebuilt per point.
 */
Eigen::Matrix<double, 3, 4> buildLidarToImageMatrix(
  const geometry_msgs::msg::TransformStamped & transform, const std::array<double, 12> & projection_matrix);

/**
 * @brief Projects a 3D LiDAR point using a precomputed lidar-to-image matrix (P * T).
 * For point loops: build the matrix once with buildLidarToImageMatrix(), then call this per point.
 * @param lidar_to_image Combined 3x4 matrix (camera projection * camera extrinsic)
 * @param pt 3D point in LiDAR frame
 * @return 2D image coordinates if projection is valid (z > min_camera_z_distance); no image-bound check (caller clips if needed).
 */
std::optional<cv::Point2d> projectLidarToCamera(
  const Eigen::Matrix<double, 3, 4> & lidar_to_image, const pcl::PointXYZ & pt);

/**
 * @brief Performs Euclidean clustering on a point cloud.
 * @param cloud Input point cloud
 * @param clusterTolerance Maximum distance between points in the same cluster
 * @param minClusterSize Minimum number of points required for a valid cluster
 * @param maxClusterSize Maximum number of points allowed in a cluster
 * @param cluster_indices Output vector of cluster indices
 */
void euclideanClusterExtraction(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double clusterTolerance,
  int minClusterSize,
  int maxClusterSize,
  std::vector<pcl::PointIndices> & cluster_indices);

/**
 * @brief Performs adaptive Euclidean clustering with distance-based tolerance.
 * @param cloud Input point cloud
 * @param base_cluster_tolerance Base cluster tolerance for far points
 * @param minClusterSize Minimum number of points required for a valid cluster
 * @param maxClusterSize Maximum number of points allowed in a cluster
 * @param cluster_indices Output vector of cluster indices
 * @param close_threshold Distance threshold to classify points as "close"
 * @param close_tolerance_mult Multiplier for cluster tolerance on close points
 * @details Uses larger cluster tolerance for closer points to prevent fragmentation.
 */
void adaptiveEuclideanClusterExtraction(
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
void assignClusterColors(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::vector<pcl::PointIndices> & cluster_indices,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & clustered_cloud);

/**
 * @brief Computes statistics for each cluster (centroid, bounds, point count)
 * @param cloud Input point cloud
 * @param cluster_indices Vector of cluster indices
 * @return Vector of cluster statistics
 */
std::vector<ClusterStats> computeClusterStats(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices);

/**
 * @brief Merges nearby clusters based on AABB gap.
 * @param cluster_indices Cluster indices (modified in place; merged clusters combined)
 * @param cloud Point cloud (for point lookups)
 * @param stats Precomputed cluster statistics (same order as cluster_indices before merge)
 * @param mergeTolerance Maximum AABB gap distance for merging
 */
void mergeClusters(
  std::vector<pcl::PointIndices> & cluster_indices,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::vector<ClusterStats> & stats,
  double mergeTolerance);

/**
 * @brief Drops cluster candidates that fail size/density/range heuristics.
 *
 * Reads thresholds from @ref getParams() (@c quality_* members). When @c candidate.match is set,
 * applies class-specific 3D caps using @p detections and the matched @c det_idx.
 *
 * @param[in,out] candidates In-place filter; order of survivors is not specified vs. input.
 * @pre @ref setParams called with valid @c quality_* fields before use.
 */
void filterCandidatesByClassAwareConstraints(
  std::vector<ClusterCandidate> & candidates, const vision_msgs::msg::Detection2DArray & detections);

/**
 * @brief Computes the centroid of a cluster
 * @param cloud Input point cloud
 * @param cluster_indices Indices of points in the cluster
 * @param centroid Output centroid point
 * @return true if centroid was computed successfully, false otherwise
 */
bool computeClusterCentroid(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const pcl::PointIndices & cluster_indices,
  pcl::PointXYZ & centroid);

/**
 * @brief Greedy one-to-one assignment: strict mode ranks surviving pairs by a combined score (IoU, in-box
 *        point fraction, AR consistency, centroid proximity, point count); legacy mode ranks by IoU only.
 *
 * When @c association_strict_matching is true: centroid in inner box, IoU on projected point rect (or AABB),
 * in-detection point fraction, tiered min points, and 2D aspect consistency must all pass. Unmatched
 * candidates are dropped. Second pass is suppressed if @c association_suppress_second_pass_under_strict.
 *
 * @param image_width Image width for projection clipping (from CameraInfo). 0 = use params or 1280.
 * @param image_height Image height for projection clipping. 0 = use params or 1024.
 */
void assignCandidatesToDetectionsByIOU(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  std::vector<ClusterCandidate> & candidates,
  const vision_msgs::msg::Detection2DArray & detections,
  const geometry_msgs::msg::TransformStamped & transform,
  const std::array<double, 12> & projection_matrix,
  float object_detection_confidence,
  int image_width = 0,
  int image_height = 0,
  const std::vector<std::optional<double>> & detection_depths = {});

/**
 * @brief Computes 3D bounding boxes for all clusters
 * @param cloud Input point cloud
 * @param cluster_indices Vector of cluster indices
 * @return Vector of 3D bounding boxes
 * @note Fitting logic lives in @c cluster_box_utils.hpp (@c cluster_box namespace).
 */
std::vector<Box3D> computeClusterBoxes(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices);

/**
 * @brief Converts precomputed boxes to visualization marker array.
 * @param boxes Precomputed 3D bounding boxes (must have same size as cluster_indices)
 * @param cluster_indices Vector of cluster indices (one-to-one with boxes)
 * @param header Frame and stamp for output markers
 * @return Marker array for RViz visualization
 */
visualization_msgs::msg::MarkerArray computeBoundingBox(
  const std::vector<Box3D> & boxes,
  const std::vector<pcl::PointIndices> & cluster_indices,
  const std_msgs::msg::Header & header);

/**
 * @brief Builds 3D detection array from boxes and candidates.
 * @param boxes 3D boxes (same order as candidates)
 * @param candidates Used for class and score via candidate.match and detections
 * @param header Frame and stamp for output
 * @param detections 2D detections (for class/scores when match.det_idx is set)
 * @return Detection3DArray with one entry per candidate (empty candidates skipped)
 */
vision_msgs::msg::Detection3DArray compute3DDetection(
  const std::vector<Box3D> & boxes,
  const std::vector<ClusterCandidate> & candidates,
  const std_msgs::msg::Header & header,
  const vision_msgs::msg::Detection2DArray & detections);

}  // namespace projection_utils

#endif
