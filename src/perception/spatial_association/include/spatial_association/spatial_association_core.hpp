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

#ifndef SPATIAL_ASSOCIATION_CORE_HPP
#define SPATIAL_ASSOCIATION_CORE_HPP

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <memory>
#include <vector>

#include "utils/projection_utils.hpp"

namespace wato::perception::spatial_association
{

/**
 * @brief Core LiDAR clustering, pre-processing, and 3D box computation logic.
 * Used by SpatialAssociationNode for voxel filtering, Euclidean clustering, cluster merging, and 3D box/centroid/color computation.
 */
class SpatialAssociationCore
{
public:
  /** One distance band for multi-band adaptive clustering. */
  struct ClusteringBand
  {
    double max_distance; /**< Upper distance bound (m) for this band. */
    double tolerance_mult; /**< Multiplier applied to base cluster tolerance. */
    int min_cluster_size; /**< Minimum cluster size for this band. */
  };

  /** Configuration for voxel grid, Euclidean clustering, and merge (quality filter lives in ProjectionUtilsParams). */
  struct ClusteringParams
  {
    float voxel_size = 0.2f;

    double euclid_cluster_tolerance = 0.5;
    int euclid_min_cluster_size = 50;
    int euclid_max_cluster_size = 700;
    bool use_adaptive_clustering = true;
    double euclid_close_threshold = 10.0;
    double euclid_close_tolerance_mult = 1.5;

    /** Multi-band clustering bands. If non-empty, overrides the legacy close/far two-band split. */
    std::vector<ClusteringBand> clustering_bands;

    double merge_threshold = 0.3;

    /** Drop points beyond this range from lidar origin (m). 0 = disabled. Reduces far noise before voxel/cluster. */
    double max_lidar_range_m = 0.0;
  };

  /**
   * @brief Constructor - initializes working PCL objects and voxel filter
   */
  SpatialAssociationCore();

  /**
   * @brief Destructor
   */
  ~SpatialAssociationCore() = default;

  /**
   * @brief Sets clustering parameters and updates voxel filter
   * @param params Clustering parameters structure
   */
  void setParams(const ClusteringParams & params);

  /**
   * @brief Gets current clustering parameters
   * @return Reference to current clustering parameters
   */
  const ClusteringParams & getParams() const
  {
    return params_;
  }

  /**
   * @brief Applies voxel downsampling to the point cloud.
   * @param input_cloud Input point cloud
   * @param output_cloud Output downsampled point cloud
   * @note Leaf size may be increased automatically for very large clouds to avoid PCL voxel index overflow.
   */
  void processPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud);

  /**
   * @brief Performs clustering on the filtered point cloud.
   * @param filtered_cloud Input filtered point cloud
   * @param cluster_indices Output vector of cluster indices
   * @details Uses Euclidean (or adaptive Euclidean) clustering. mergeClusters is temporarily disabled
   *          in the implementation; re-enable in spatial_association_core.cpp when needed.
   *          Post-IoU quality filtering runs in the node via
   *          @c projection_utils::filterCandidatesByClassAwareConstraints (uses @c projection_utils::getParams()).
   */
  void performClustering(
    pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud, std::vector<pcl::PointIndices> & cluster_indices);

  /**
   * @brief Computes 3D bounding boxes for clusters
   * @param filtered_cloud Input filtered point cloud
   * @param cluster_indices Vector of cluster indices
   * @return Vector of 3D bounding boxes
   */
  std::vector<projection_utils::Box3D> computeClusterBoxes(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud,
    const std::vector<pcl::PointIndices> & cluster_indices) const;

  /**
   * @brief Assigns random colors to clusters for visualization
   * @param filtered_cloud Input filtered point cloud
   * @param cluster_indices Vector of cluster indices
   * @param colored_cluster Output colored point cloud
   */
  void assignClusterColors(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud,
    const std::vector<pcl::PointIndices> & cluster_indices,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & colored_cluster) const;

  /**
   * @brief Computes centroids for all clusters
   * @param filtered_cloud Input filtered point cloud
   * @param cluster_indices Vector of cluster indices
   * @param centroid_cloud Output point cloud containing cluster centroids
   */
  void computeClusterCentroids(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud,
    const std::vector<pcl::PointIndices> & cluster_indices,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & centroid_cloud) const;

private:
  ClusteringParams params_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_colored_cluster_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr working_centroid_cloud_;
};

}  // namespace wato::perception::spatial_association

#endif  // SPATIAL_ASSOCIATION_CORE_HPP
