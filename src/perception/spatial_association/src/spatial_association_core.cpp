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

#include "spatial_association/spatial_association_core.hpp"

#include <algorithm>
#include <limits>
#include <vector>

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> nan_removed_idx;
  pcl::removeNaNFromPointCloud(*input_cloud, *cleaned, nan_removed_idx);
  if (!cleaned || cleaned->empty()) {
    if (output_cloud) {
      output_cloud->clear();
    }
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr working = cleaned;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ranged_holder;
  if (params_.max_lidar_range_m > 0.0) {
    const double rmax = params_.max_lidar_range_m;
    const double r2 = rmax * rmax;
    ranged_holder.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ranged_holder->reserve(cleaned->size());
    for (const auto & p : cleaned->points) {
      const double d2 =
        static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z;
      if (d2 <= r2 && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        ranged_holder->points.push_back(p);
      }
    }
    ranged_holder->width = ranged_holder->points.size();
    ranged_holder->height = 1;
    if (ranged_holder->empty()) {
      if (output_cloud) {
        output_cloud->clear();
      }
      return;
    }
    working = ranged_holder;
  }

  // PCL VoxelGrid uses a single int for voxel index; too many voxels overflow.
  // Clamp leaf size so extent/leaf_size <= kMaxVoxelsPerDimension (~safe for 32-bit index).
  float min_x = std::numeric_limits<float>::max();
  float min_y = min_x, min_z = min_x;
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = max_x, max_z = max_x;
  for (const auto & p : working->points) {
    min_x = std::min(min_x, p.x);
    min_y = std::min(min_y, p.y);
    min_z = std::min(min_z, p.z);
    max_x = std::max(max_x, p.x);
    max_y = std::max(max_y, p.y);
    max_z = std::max(max_z, p.z);
  }
  const float extent = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
  // PCL uses 32-bit voxel index; ~1200 voxels/dim is safe and keeps resolution when clamping.
  constexpr float kMaxVoxelsPerDimension = 1200.f;
  const float min_leaf = extent / kMaxVoxelsPerDimension;
  const float leaf = std::max(params_.voxel_size, min_leaf);
  voxel_filter_.setLeafSize(leaf, leaf, leaf);

  voxel_filter_.setInputCloud(working);
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
  // FLANN KdTree used by EuclideanClusterExtraction can assert on very small or degenerate clouds.
  if (filtered_cloud->size() < 2u) {
    return;
  }

  if (!params_.clustering_bands.empty()) {
    // Multi-band adaptive clustering: N distance bands with per-band tolerance and min size.
    std::vector<projection_utils::ClusteringBand> bands;
    bands.reserve(params_.clustering_bands.size());
    for (const auto & b : params_.clustering_bands) {
      bands.push_back({b.max_distance, b.tolerance_mult, b.min_cluster_size});
    }
    projection_utils::multiBandClusterExtraction(
      filtered_cloud,
      params_.euclid_cluster_tolerance,
      params_.euclid_max_cluster_size,
      cluster_indices,
      bands);
  } else if (params_.use_adaptive_clustering) {
    projection_utils::adaptiveEuclideanClusterExtraction(
      filtered_cloud,
      params_.euclid_cluster_tolerance,
      params_.euclid_min_cluster_size,
      params_.euclid_max_cluster_size,
      cluster_indices,
      params_.euclid_close_threshold,
      params_.euclid_close_tolerance_mult);
  } else {
    projection_utils::euclideanClusterExtraction(
      filtered_cloud,
      params_.euclid_cluster_tolerance,
      params_.euclid_min_cluster_size,
      params_.euclid_max_cluster_size,
      cluster_indices);
  }

  // TEMP: post-cluster merge disabled. Re-enable by uncommenting the block below.
  // Computes stats and merges nearby clusters (AABB gap < merge_threshold). Physics filtering is done
  // in the node on ClusterCandidates.
  // if (!cluster_indices.empty()) {
  //   auto cluster_stats = projection_utils::computeClusterStats(filtered_cloud, cluster_indices);
  //   projection_utils::mergeClusters(cluster_indices, filtered_cloud, cluster_stats, params_.merge_threshold);
  // }
}

std::vector<projection_utils::Box3D> SpatialAssociationCore::computeClusterBoxes(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud,
  const std::vector<pcl::PointIndices> & cluster_indices) const
{
  return projection_utils::computeClusterBoxes(filtered_cloud, cluster_indices);
}

void SpatialAssociationCore::assignClusterColors(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud,
  const std::vector<pcl::PointIndices> & cluster_indices,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & colored_cluster) const
{
  projection_utils::assignClusterColors(filtered_cloud, cluster_indices, colored_cluster);
}

void SpatialAssociationCore::computeClusterCentroids(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud,
  const std::vector<pcl::PointIndices> & cluster_indices,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & centroid_cloud) const
{
  centroid_cloud->clear();
  for (const auto & ci : cluster_indices) {
    pcl::PointXYZ c;
    projection_utils::computeClusterCentroid(filtered_cloud, ci, c);
    centroid_cloud->points.push_back(c);
  }
  centroid_cloud->width = centroid_cloud->points.size();
  centroid_cloud->height = 1;
  centroid_cloud->is_dense = true;
}
