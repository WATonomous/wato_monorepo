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

#ifndef CLUSTER_BOX_UTILS_HPP
#define CLUSTER_BOX_UTILS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <vector>

#include "utils/projection_utils.hpp"

/**
 * @brief 3D oriented box fitting and percentile-based cluster bounds from LiDAR points.
 *
 * Used by @c projection_utils for @c computeClusterBoxes, @c computeClusterStats, and
 * @c mergeClusters. Parameters come from @c projection_utils::getParams() (set via ROS).
 */
namespace cluster_box
{

/** Oriented 3D box from cluster points (xy from search + percentiles, z from percentiles). */
projection_utils::Box3D computeClusterBox(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::PointIndices & cluster);

/** Centroid + percentile-trimmed axis-aligned bounds for one cluster. */
projection_utils::ClusterStats computeSingleClusterStats(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::PointIndices & indices);

/** Per-cluster stats for many clusters (same semantics as @c projection_utils::computeClusterStats). */
std::vector<projection_utils::ClusterStats> computeClusterStatsBatch(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices);

}  // namespace cluster_box

#endif  // CLUSTER_BOX_UTILS_HPP
