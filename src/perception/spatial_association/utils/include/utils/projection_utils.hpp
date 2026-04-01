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
#include <unordered_map>
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
namespace wato::perception::projection_utils
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

  /** If true, use L-shaped fitting for vehicle classes (car/truck/bus) instead of search-based fit. */
  bool use_l_shaped_fitting = true;

  double min_camera_z_distance = 1.0;

  int image_width = 0;
  int image_height = 0;

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
  int quality_min_points_default = 8;
  /** Min points when centroid distance exceeds @ref quality_distance_threshold_far. */
  int quality_min_points_far = 8;
  /** Min points when distance exceeds @ref quality_distance_threshold_medium (but not far). */
  int quality_min_points_medium = 10;
  /** Min points when AABB volume exceeds @ref quality_volume_threshold_large (near range). */
  int quality_min_points_large = 30;
  double quality_distance_threshold_far = 30.0;
  double quality_distance_threshold_medium = 20.0;
  float quality_volume_threshold_large = 8.0f;
  /** Minimum points-per-unit-volume when volume is non-trivial. */
  float quality_min_density = 5.0f;
  /** Max largest AABB edge length (m). */
  float quality_max_dimension = 15.0f;
  /** Max ratio of largest to smallest AABB edge (thin slivers rejected). */
  float quality_max_aspect_ratio = 15.0f;
  /** @} */

  /** @name Strict association (primary IoU pass)
   *  When @c association_strict_matching is true, a candidate pairs with a detection only if all
   *  gates pass (inner-box centroid, IoU on projected point hull or AABB, in-box point fraction,
   *  tiered min points, 2D aspect consistency).
   */
  bool association_strict_matching = true;
  double association_centroid_inner_box_fraction = 0.85;
  double association_min_inside_point_fraction = 0.45;
  double association_min_ar_consistency_score = 0.30;
  bool association_use_point_projection_rect_for_iou = true;

  /** Association scoring weights (must sum to 1.0 before depth adjustment). */
  double association_weight_iou = 0.30;
  double association_weight_inside_fraction = 0.28;
  double association_weight_ar = 0.18;
  double association_weight_centroid = 0.14;
  double association_weight_points = 0.10;

  /** Class-aware threshold scaling (applied on top of distance-adaptive scaling). */
  double person_inside_fraction_scale = 0.85;
  double vehicle_inside_fraction_scale = 0.85;
  double person_iou_threshold_scale = 0.90;
  double vehicle_iou_threshold_scale = 0.90;
  /** AR consistency gate multiplier for persons (projection shape is unreliable for narrow objects). */
  double person_ar_consistency_scale = 0.70;

  /** Absolute minimum inside-point count (prevents spurious single-point matches). */
  int association_min_inside_points = 2;
  /** Size consistency: min(w_ratio, 1/w_ratio) * min(h_ratio, 1/h_ratio) floor. Rejects wrong-scale clusters. */
  double min_size_consistency_score = 0.25;
  /** Min 2D detection area (px^2) for clusters beyond far distance threshold. 0 = disabled. */
  double min_det_area_far_px2 = 400.0;

  /** Centroid score: detection scale = this * hypot(box_w, box_h). Controls centroid proximity sensitivity. */
  double centroid_score_detection_scale = 0.35;
  /** Point score saturates at this many points (log-scale). */
  double point_score_saturation_count = 120.0;
  /** Early rejection: skip candidate-detection pairs where centroid is > this * max(det_w, det_h) away. */
  double association_centroid_distance_multiplier = 1.5;
  /** When only 1 point projects to image, create a synthetic bbox with this half-size (pixels). */
  double single_point_bbox_margin_px = 3.0;

  /** Minimum enriched depth (m) to accept as valid; depths below this are rejected. */
  double min_depth_for_enrichment = 0.1;

  /** L-shape energy: combined = mean_edge_dist + this * variance. */
  double l_shape_energy_variance_weight = 0.5;
  /** Orientation search: coarse step = this * fine step. */
  double orientation_coarse_step_multiplier = 5.0;
  /** Orientation search: fine range = this * fine step around best coarse angle. */
  double orientation_fine_range_multiplier = 2.5;

  /** Weight of the depth-proximity term when enriched 3D detections are available. */
  double depth_score_weight = 0.15;
  /** Scale (metres) for depth score: exp(-|delta| / scale). */
  double depth_score_scale = 5.0;

  /** @{ @name Class-aware size prior (pre-match penalty)
   *  Penalizes candidate-detection pairs where the cluster's 3D AABB dimensions deviate from expected
   *  class dimensions. Applied as an exponential decay before depth scoring.
   */
  struct ClassSizePrior
  {
    double min_width{0.0};  ///< Min footprint along shorter horizontal axis (m).
    double max_width{100.0};
    double min_length{0.0};  ///< Min footprint along longer horizontal axis (m).
    double max_length{100.0};
    double min_height{0.0};
    double max_height{100.0};
  };

  bool enable_size_prior = true;  ///< Master toggle for class-specific size prior scoring.
  double size_prior_weight = 0.10;  ///< Weight of size-prior term (0 = disabled).
  double size_prior_scale = 0.5;  ///< Exponential decay scale (m); lower = stricter.
  std::unordered_map<std::string, ClassSizePrior> size_priors;
  /** @} */

  /** If true, use Hungarian (optimal) assignment instead of greedy. */
  bool use_hungarian_assignment = true;

  /** @{ @name Distance-adaptive thresholds
   *  Scale min_iou_threshold and min_inside_point_fraction by cluster distance to improve far-range recall.
   */
  double far_iou_threshold_scale = 0.5;
  double medium_iou_threshold_scale = 0.75;
  double far_inside_fraction_scale = 0.6;
  double medium_inside_fraction_scale = 0.8;
  /** @} */

  /** Class-tight 3D caps after match (meters); applied in @ref filterCandidatesByClassAwareConstraints. */
  float quality_person_max_height_m = 2.5f;
  float quality_person_max_footprint_xy_m = 1.45f;
  /** Extra class-aware guard: reject oversized person clusters by AABB volume (m^3). */
  float quality_person_max_volume_m3 = 4.0f;
  float quality_vehicle_max_height_m = 4.8f;
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

/** One distance band for multi-band clustering. */
struct ClusteringBand
{
  double max_distance;
  double tolerance_mult;
  int min_cluster_size;
};

/**
 * @brief Multi-band adaptive clustering: splits cloud into N distance bands, each with its own
 *        tolerance and min cluster size. Generalizes the two-band adaptive clustering.
 * @param cloud Input point cloud
 * @param base_cluster_tolerance Base tolerance (each band multiplies this)
 * @param maxClusterSize Maximum cluster size for all bands
 * @param cluster_indices Output cluster indices (indices into original cloud)
 * @param bands Sorted vector of bands (ascending max_distance)
 */
void multiBandClusterExtraction(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double base_cluster_tolerance,
  int maxClusterSize,
  std::vector<pcl::PointIndices> & cluster_indices,
  const std::vector<ClusteringBand> & bands);

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

/** 2D axis-aligned bounding box for BEV deduplication. */
struct Aabb2d
{
  double minx{0.0};
  double maxx{0.0};
  double miny{0.0};
  double maxy{0.0};
};

/** Work item for cross-camera deduplication: one 3D detection with optional marker and LiDAR indices. */
struct CrossCameraWorkItem
{
  vision_msgs::msg::Detection3D det3d;
  visualization_msgs::msg::Marker bbox;
  bool has_bbox{false};
  std::vector<int> sorted_indices;
  double score{0.0};
};

/** Sort and deduplicate an index vector in-place. */
void sortUniqueIndicesInPlace(std::vector<int> & v);

/** BEV axis-aligned bounding box from an oriented 3D bounding box (projects xy footprint). */
Aabb2d bevAabbFromBoundingBox3D(const vision_msgs::msg::BoundingBox3D & b);

/** Compute IoU of two 2D AABBs. */
double bevAabbIou(const Aabb2d & a, const Aabb2d & b);

/**
 * @brief Check whether two cross-camera work items are duplicates.
 * Uses LiDAR index overlap, class match, BEV center distance, and BEV box IoU.
 */
bool areCrossCameraDuplicates(
  const CrossCameraWorkItem & a,
  const CrossCameraWorkItem & b,
  double min_overlap,
  double weak_overlap,
  double max_center_m,
  double min_bev_iou,
  double min_bev_iou_no_class);

/**
 * @brief Suppress duplicate detections across cameras by score-ordered NMS.
 * Higher-scoring items suppress lower-scoring duplicates (determined by @ref areCrossCameraDuplicates).
 */
void deduplicateCrossCameraWorkItems(
  std::vector<CrossCameraWorkItem> & items,
  double min_overlap,
  double weak_overlap,
  double max_center_m,
  double min_bev_iou,
  double min_bev_iou_no_class);

}  // namespace wato::perception::projection_utils

#endif
