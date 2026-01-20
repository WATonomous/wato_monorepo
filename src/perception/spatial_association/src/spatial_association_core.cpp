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

#include "spatial_association_core.hpp"

SpatialAssociationCore::SpatialAssociationCore()
{
  working_colored_cluster_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  working_centroid_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  voxel_filter_.setLeafSize(0.2f, 0.2f, 0.2f);
}

void SpatialAssociationCore::setParams(const ClusteringParams & params)
{
  params_ = params;
  voxel_filter_.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
}

void SpatialAssociationCore::processPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud)
{
  if (!input_cloud || input_cloud->empty()) {
    if (output_cloud) {
      output_cloud->clear();
    }
    return;
  }

  // Filter directly from input to output, eliminating unnecessary copies
  voxel_filter_.setInputCloud(input_cloud);
  voxel_filter_.filter(*output_cloud);
}

void SpatialAssociationCore::performClustering(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud, std::vector<pcl::PointIndices> & cluster_indices)
{
  cluster_indices.clear();
  // Reserve memory to avoid reallocations during clustering operations
  cluster_indices.reserve(100);  // Reasonable estimate for typical scene

  if (!filtered_cloud || filtered_cloud->empty()) {
    return;
  }

  if (params_.use_adaptive_clustering) {
    ProjectionUtils::adaptiveEuclideanClusterExtraction(
      filtered_cloud,
      params_.euclid_cluster_tolerance,
      params_.euclid_min_cluster_size,
      params_.euclid_max_cluster_size,
      cluster_indices,
      params_.euclid_close_threshold,
      params_.euclid_close_tolerance_mult);
  } else {
    ProjectionUtils::euclideanClusterExtraction(
      filtered_cloud,
      params_.euclid_cluster_tolerance,
      params_.euclid_min_cluster_size,
      params_.euclid_max_cluster_size,
      cluster_indices);
  }

  // Compute stats once after clustering
  auto cluster_stats = ProjectionUtils::computeClusterStats(filtered_cloud, cluster_indices);

  // Filter ground noise (modifies cluster_indices)
  ProjectionUtils::filterGroundNoise(cluster_stats, cluster_indices);

  // Only continue if clusters remain after ground noise filtering
  if (!cluster_indices.empty()) {
    // Recompute stats after filterGroundNoise modifies cluster_indices
    cluster_stats = ProjectionUtils::computeClusterStats(filtered_cloud, cluster_indices);

    // Step 4: Physics-based quality filtering with distance-adaptive thresholds
    ProjectionUtils::filterClustersByPhysicsConstraints(
      cluster_stats,
      cluster_indices,
      params_.max_distance,
      params_.min_points,
      params_.min_height,
      params_.min_points_default,
      params_.min_points_far,
      params_.min_points_medium,
      params_.min_points_large,
      params_.distance_threshold_far,
      params_.distance_threshold_medium,
      params_.volume_threshold_large,
      params_.min_density,
      params_.max_density,
      params_.max_dimension,
      params_.max_aspect_ratio);

    // Only merge if clusters remain after quality filtering
    if (!cluster_indices.empty()) {
      // Note: mergeClusters uses stats for centroid distances, but modifies cluster_indices
      // Stats may be slightly stale after merging, but merge threshold is small enough
      // that this is acceptable for performance
      ProjectionUtils::mergeClusters(cluster_indices, filtered_cloud, cluster_stats, params_.merge_threshold);
    }
  }
}

std::vector<ProjectionUtils::Box3D> SpatialAssociationCore::computeClusterBoxes(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud,
  const std::vector<pcl::PointIndices> & cluster_indices) const
{
  return ProjectionUtils::computeClusterBoxes(filtered_cloud, cluster_indices);
}

void SpatialAssociationCore::assignClusterColors(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud,
  const std::vector<pcl::PointIndices> & cluster_indices,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & colored_cluster) const
{
  ProjectionUtils::assignClusterColors(filtered_cloud, cluster_indices, colored_cluster);
}

void SpatialAssociationCore::computeClusterCentroids(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud,
  const std::vector<pcl::PointIndices> & cluster_indices,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & centroid_cloud) const
{
  centroid_cloud->clear();
  for (const auto & ci : cluster_indices) {
    pcl::PointXYZ c;
    ProjectionUtils::computeClusterCentroid(filtered_cloud, ci, c);
    centroid_cloud->points.push_back(c);
  }
  centroid_cloud->width = centroid_cloud->points.size();
  centroid_cloud->height = 1;
  centroid_cloud->is_dense = true;
}
