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

#include <rclcpp/rclcpp.hpp>

#include "utils/cluster_box_utils.hpp"

namespace
{
const auto kLogger = rclcpp::get_logger("spatial_association_core");

void mergeVerticallyAlignedClusters(
  std::vector<pcl::PointIndices> & cluster_indices,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  std::vector<projection_utils::ClusterStats> & stats,
  double max_xy_centroid_dist)
{
  if (!cloud || cluster_indices.size() < 2u || stats.size() != cluster_indices.size()) {
    return;
  }

  std::vector<bool> merged(cluster_indices.size(), false);
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (merged[i]) continue;
    for (size_t j = i + 1; j < cluster_indices.size(); ++j) {
      if (merged[j]) continue;

      const double dx = static_cast<double>(stats[i].centroid.x() - stats[j].centroid.x());
      const double dy = static_cast<double>(stats[i].centroid.y() - stats[j].centroid.y());
      const double xy_dist = std::hypot(dx, dy);
      if (xy_dist > max_xy_centroid_dist) continue;

      const double z_min_i = static_cast<double>(stats[i].min_z);
      const double z_max_i = static_cast<double>(stats[i].max_z);
      const double z_min_j = static_cast<double>(stats[j].min_z);
      const double z_max_j = static_cast<double>(stats[j].max_z);
      const double z_gap = std::max(0.0, std::max(z_min_i, z_min_j) - std::min(z_max_i, z_max_j));
      if (z_gap > 0.2) continue;  // Keep vertical merge tight to avoid cross-object fusion.

      const double x_overlap = std::max(
        0.0,
        std::min(static_cast<double>(stats[i].max_x), static_cast<double>(stats[j].max_x)) -
          std::max(static_cast<double>(stats[i].min_x), static_cast<double>(stats[j].min_x)));
      const double y_overlap = std::max(
        0.0,
        std::min(static_cast<double>(stats[i].max_y), static_cast<double>(stats[j].max_y)) -
          std::max(static_cast<double>(stats[i].min_y), static_cast<double>(stats[j].min_y)));
      const double area_i = std::max(
        1e-6,
        (static_cast<double>(stats[i].max_x) - static_cast<double>(stats[i].min_x)) *
          (static_cast<double>(stats[i].max_y) - static_cast<double>(stats[i].min_y)));
      const double area_j = std::max(
        1e-6,
        (static_cast<double>(stats[j].max_x) - static_cast<double>(stats[j].min_x)) *
          (static_cast<double>(stats[j].max_y) - static_cast<double>(stats[j].min_y)));
      const double overlap_ratio = (x_overlap * y_overlap) / std::max(1e-6, std::min(area_i, area_j));
      if (overlap_ratio < 0.3) continue;

      cluster_indices[i].indices.insert(
        cluster_indices[i].indices.end(), cluster_indices[j].indices.begin(), cluster_indices[j].indices.end());
      merged[j] = true;
      stats[i] = cluster_box::computeSingleClusterStats(cloud, cluster_indices[i]);
    }
  }

  std::vector<pcl::PointIndices> kept_clusters;
  std::vector<projection_utils::ClusterStats> kept_stats;
  kept_clusters.reserve(cluster_indices.size());
  kept_stats.reserve(stats.size());
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (merged[i]) continue;
    kept_stats.push_back(stats[i]);
    kept_clusters.push_back(std::move(cluster_indices[i]));
  }
  cluster_indices = std::move(kept_clusters);
  stats = std::move(kept_stats);
}
}

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
      const double d2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y +
        static_cast<double>(p.z) * p.z;
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

  if (params_.skip_voxel_for_roi_first) {
    *output_cloud = *working;
    return;
  }

  // PCL VoxelGrid uses 32-bit indexing internally; too many voxels can overflow.
  // Instead of clamping by max extent only (overly conservative on long/highway scenes),
  // clamp by estimated total voxel index count nx*ny*nz.
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
  const double ex = std::max(1e-3, static_cast<double>(max_x - min_x));
  const double ey = std::max(1e-3, static_cast<double>(max_y - min_y));
  const double ez = std::max(1e-3, static_cast<double>(max_z - min_z));
  const double configured_leaf = std::max(1e-4, static_cast<double>(params_.voxel_size));
  constexpr double kMaxVoxelIndexProduct = static_cast<double>(std::numeric_limits<int32_t>::max());
  auto estimatedVoxelProduct = [&](double leaf) -> double {
    const double nx = std::floor(ex / leaf) + 1.0;
    const double ny = std::floor(ey / leaf) + 1.0;
    const double nz = std::floor(ez / leaf) + 1.0;
    return nx * ny * nz;
  };

  double effective_leaf = configured_leaf;
  if (estimatedVoxelProduct(effective_leaf) > kMaxVoxelIndexProduct) {
    double low = configured_leaf;
    double high = std::max({ex, ey, ez, configured_leaf});
    for (int iter = 0; iter < 40; ++iter) {
      const double mid = 0.5 * (low + high);
      if (estimatedVoxelProduct(mid) > kMaxVoxelIndexProduct) {
        low = mid;
      } else {
        high = mid;
      }
    }
    effective_leaf = high;
  }
  const float leaf = static_cast<float>(effective_leaf);
  if (leaf > params_.voxel_size + 1e-6f) {
    RCLCPP_WARN(
      kLogger,
      "Voxel leaf clamped from configured %.3f m to %.3f m due to voxel-index safety "
      "(extents x=%.1f m y=%.1f m z=%.1f m). Clustering behavior may differ from tuned settings.",
      params_.voxel_size,
      leaf,
      ex,
      ey,
      ez);
  }
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

  if (params_.use_adaptive_clustering) {
    projection_utils::adaptiveEuclideanClusterExtraction(
      filtered_cloud,
      params_.euclid_cluster_tolerance,
      params_.euclid_min_cluster_size,
      params_.euclid_max_cluster_size,
      cluster_indices,
      params_.euclid_close_threshold,
      params_.euclid_mid_threshold,
      params_.euclid_near_tolerance_mult,
      params_.euclid_mid_tolerance_mult,
      params_.euclid_far_tolerance_mult,
      params_.euclid_band_near_mid_overlap_m,
      params_.euclid_band_mid_far_overlap_m,
      params_.euclid_band_merge_min_index_overlap);
  } else {
    projection_utils::euclideanClusterExtraction(
      filtered_cloud,
      params_.euclid_cluster_tolerance,
      params_.euclid_min_cluster_size,
      params_.euclid_max_cluster_size,
      cluster_indices);
  }

  if (!cluster_indices.empty()) {
    auto cluster_stats = projection_utils::computeClusterStats(filtered_cloud, cluster_indices);

    // Pass 1: merge by AABB gap distance.
    projection_utils::mergeClusters(cluster_indices, filtered_cloud, cluster_stats, params_.merge_threshold);

    // Refresh stats after pass 1 before vertical-alignment pass.
    cluster_stats = projection_utils::computeClusterStats(filtered_cloud, cluster_indices);

    // Pass 2: merge clusters split vertically by scan pattern.
    mergeVerticallyAlignedClusters(cluster_indices, filtered_cloud, cluster_stats, 0.2);
  }
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
