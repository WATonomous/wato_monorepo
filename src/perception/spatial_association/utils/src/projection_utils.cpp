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

#include "utils/projection_utils.hpp"

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "pcl/filters/voxel_grid.h"
#include "utils/cluster_box_utils.hpp"
#include "utils/hungarian.hpp"
namespace projection_utils
{

static ProjectionUtilsParams s_params_{};

void setParams(const ProjectionUtilsParams & params)
{
  s_params_ = params;
}

const ProjectionUtilsParams & getParams()
{
  return s_params_;
}

std::vector<ClusterStats> computeClusterStats(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices)
{
  return cluster_box::computeClusterStatsBatch(cloud, cluster_indices);
}

std::vector<ClusterCandidate> buildCandidates(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices)
{
  std::vector<ClusterCandidate> candidates;
  candidates.reserve(cluster_indices.size());
  std::vector<ClusterStats> stats = cluster_box::computeClusterStatsBatch(cloud, cluster_indices);
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    candidates.push_back(ClusterCandidate{cluster_indices[i], std::move(stats[i]), std::nullopt, std::nullopt});
  }
  return candidates;
}

std::vector<pcl::PointIndices> extractIndices(const std::vector<ClusterCandidate> & candidates)
{
  std::vector<pcl::PointIndices> indices;
  indices.reserve(candidates.size());
  for (const auto & c : candidates) {
    indices.push_back(c.indices);
  }
  return indices;
}

std::vector<Box3D> computeClusterBoxes(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices)
{
  std::vector<Box3D> boxes;
  boxes.reserve(cluster_indices.size());
  for (const auto & c : cluster_indices) {
    boxes.push_back(cluster_box::computeClusterBox(cloud, c));
  }
  return boxes;
}

namespace
{
Eigen::Matrix4d transformToMatrix4d(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & t = transform.transform.translation;
  const auto & r = transform.transform.rotation;
  Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  T(0, 3) = t.x;
  T(1, 3) = t.y;
  T(2, 3) = t.z;
  return T;
}

Eigen::Matrix<double, 3, 4> arrayToProjection3x4(const std::array<double, 12> & p)
{
  Eigen::Matrix<double, 3, 4> P;
  P << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11];
  return P;
}
}  // namespace

Eigen::Matrix<double, 3, 4> buildLidarToImageMatrix(
  const geometry_msgs::msg::TransformStamped & transform, const std::array<double, 12> & projection_matrix)
{
  const Eigen::Matrix4d T = transformToMatrix4d(transform);
  const Eigen::Matrix<double, 3, 4> P = arrayToProjection3x4(projection_matrix);
  return P * T;
}

std::optional<cv::Point2d> projectLidarToCamera(
  const Eigen::Matrix<double, 3, 4> & lidar_to_image, const pcl::PointXYZ & pt)
{
  Eigen::Vector4d lidar_pt(pt.x, pt.y, pt.z, 1.0);
  Eigen::Vector3d projected = lidar_to_image * lidar_pt;
  if (projected.z() <= 0.0 || projected.z() < getParams().min_camera_z_distance) {
    return std::nullopt;
  }
  cv::Point2d proj_pt;
  proj_pt.x = projected.x() / projected.z();
  proj_pt.y = projected.y() / projected.z();
  return proj_pt;
}

void euclideanClusterExtraction(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double clusterTolerance,
  int minClusterSize,
  int maxClusterSize,
  std::vector<pcl::PointIndices> & cluster_indices)
{
  cluster_indices.clear();
  if (!cloud || cloud->size() < 2u) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> original_index;
  dense_cloud->reserve(cloud->size());
  original_index.reserve(cloud->size());
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto & pt = cloud->points[i];
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
      dense_cloud->points.push_back(pt);
      original_index.push_back(static_cast<int>(i));
    }
  }
  dense_cloud->width = dense_cloud->points.size();
  dense_cloud->height = 1;
  dense_cloud->is_dense = true;
  if (dense_cloud->size() < 2u) {
    return;
  }
  std::vector<pcl::PointIndices> dense_clusters;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(dense_cloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minClusterSize);
  ec.setMaxClusterSize(maxClusterSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(dense_cloud);
  ec.extract(dense_clusters);
  cluster_indices.reserve(dense_clusters.size());
  for (const auto & dc : dense_clusters) {
    pcl::PointIndices orig;
    orig.indices.reserve(dc.indices.size());
    for (int idx : dc.indices) {
      orig.indices.push_back(original_index[idx]);
    }
    cluster_indices.push_back(orig);
  }
}

static void appendEuclideanClustersMapped(
  pcl::PointCloud<pcl::PointXYZ>::Ptr segment,
  const std::vector<int> & orig_indices,
  double tolerance,
  int minClusterSize,
  int maxClusterSize,
  std::vector<pcl::PointIndices> & cluster_indices)
{
  if (!segment || segment->size() < 2u) {
    return;
  }
  std::vector<pcl::PointIndices> local;
  euclideanClusterExtraction(segment, tolerance, minClusterSize, maxClusterSize, local);
  for (auto & cluster : local) {
    pcl::PointIndices mapped;
    mapped.indices.reserve(cluster.indices.size());
    for (int idx : cluster.indices) {
      mapped.indices.push_back(orig_indices[static_cast<size_t>(idx)]);
    }
    cluster_indices.push_back(std::move(mapped));
  }
}

static size_t countSortedIntersection(const std::vector<int> & a, const std::vector<int> & b)
{
  size_t i = 0;
  size_t j = 0;
  size_t cnt = 0;
  while (i < a.size() && j < b.size()) {
    if (a[i] == b[j]) {
      ++cnt;
      ++i;
      ++j;
    } else if (a[i] < b[j]) {
      ++i;
    } else {
      ++j;
    }
  }
  return cnt;
}

/** Union clusters that are mostly the same points (e.g. same object extracted in overlapping radial bands). */
static void mergeClustersBySortedIndexOverlap(
  std::vector<pcl::PointIndices> & cluster_indices,
  double min_overlap_ratio,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud = nullptr,
  double max_centroid_xy_dist = 0.0,
  double min_z_overlap_for_centroid_merge = 0.0)
{
  if (cluster_indices.size() < 2 || min_overlap_ratio <= 0.0) {
    return;
  }

  const size_t n = cluster_indices.size();
  for (auto & c : cluster_indices) {
    std::sort(c.indices.begin(), c.indices.end());
  }

  struct ClusterGeom
  {
    bool valid = false;
    double cx = 0.0;
    double cy = 0.0;
    double z_min = 0.0;
    double z_max = 0.0;
  };
  std::vector<ClusterGeom> geoms(n);
  const bool enable_centroid_fallback =
    cloud && max_centroid_xy_dist > 0.0 && min_z_overlap_for_centroid_merge > 0.0;
  if (enable_centroid_fallback) {
    for (size_t i = 0; i < n; ++i) {
      const auto & idxs = cluster_indices[i].indices;
      if (idxs.empty()) continue;

      double sx = 0.0;
      double sy = 0.0;
      double z0 = std::numeric_limits<double>::infinity();
      double z1 = -std::numeric_limits<double>::infinity();
      size_t count = 0;
      for (int idx : idxs) {
        if (idx < 0 || static_cast<size_t>(idx) >= cloud->points.size()) continue;
        const auto & p = cloud->points[static_cast<size_t>(idx)];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        sx += static_cast<double>(p.x);
        sy += static_cast<double>(p.y);
        z0 = std::min(z0, static_cast<double>(p.z));
        z1 = std::max(z1, static_cast<double>(p.z));
        ++count;
      }
      if (count == 0) continue;
      geoms[i].valid = true;
      geoms[i].cx = sx / static_cast<double>(count);
      geoms[i].cy = sy / static_cast<double>(count);
      geoms[i].z_min = z0;
      geoms[i].z_max = z1;
    }
  }

  std::vector<int> parent(n);
  std::iota(parent.begin(), parent.end(), 0);

  std::function<int(int)> find = [&](int x) -> int {
    if (parent[static_cast<size_t>(x)] != x) {
      parent[static_cast<size_t>(x)] = find(parent[static_cast<size_t>(x)]);
    }
    return parent[static_cast<size_t>(x)];
  };

  auto unite = [&](int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b) {
      parent[static_cast<size_t>(a)] = b;
    }
  };

  for (size_t i = 0; i < n; ++i) {
    const auto & ai = cluster_indices[i].indices;
    if (ai.empty()) {
      continue;
    }
    const size_t si = ai.size();
    for (size_t j = i + 1; j < n; ++j) {
      const auto & aj = cluster_indices[j].indices;
      if (aj.empty()) {
        continue;
      }
      const size_t sj = aj.size();
      const size_t inter = countSortedIntersection(ai, aj);
      const size_t mn = std::min(si, sj);
      if (mn == 0) {
        continue;
      }
      if (static_cast<double>(inter) / static_cast<double>(mn) >= min_overlap_ratio) {
        unite(static_cast<int>(i), static_cast<int>(j));
      } else if (enable_centroid_fallback && geoms[i].valid && geoms[j].valid) {
        const double centroid_xy_dist = std::hypot(geoms[i].cx - geoms[j].cx, geoms[i].cy - geoms[j].cy);
        const double z_inter = std::max(0.0, std::min(geoms[i].z_max, geoms[j].z_max) - std::max(geoms[i].z_min, geoms[j].z_min));
        const double z_min_span = std::max(
          1e-6, std::min(geoms[i].z_max - geoms[i].z_min, geoms[j].z_max - geoms[j].z_min));
        const double z_overlap = z_inter / z_min_span;
        if (centroid_xy_dist < max_centroid_xy_dist && z_overlap > min_z_overlap_for_centroid_merge) {
          unite(static_cast<int>(i), static_cast<int>(j));
        }
      }
    }
  }

  std::unordered_map<int, std::vector<int>> groups;
  for (size_t i = 0; i < n; ++i) {
    const int r = find(static_cast<int>(i));
    auto & g = groups[r];
    g.insert(g.end(), cluster_indices[i].indices.begin(), cluster_indices[i].indices.end());
  }

  std::vector<pcl::PointIndices> merged;
  merged.reserve(groups.size());
  for (auto & kv : groups) {
    auto & idxs = kv.second;
    std::sort(idxs.begin(), idxs.end());
    idxs.erase(std::unique(idxs.begin(), idxs.end()), idxs.end());
    pcl::PointIndices pi;
    pi.indices = std::move(idxs);
    merged.push_back(std::move(pi));
  }
  cluster_indices = std::move(merged);
}

void adaptiveEuclideanClusterExtraction(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double base_cluster_tolerance,
  int minClusterSize,
  int maxClusterSize,
  std::vector<pcl::PointIndices> & cluster_indices,
  double close_threshold,
  double mid_threshold,
  double near_tolerance_mult,
  double mid_tolerance_mult,
  double far_tolerance_mult,
  double band_near_mid_overlap_m,
  double band_mid_far_overlap_m,
  double band_merge_min_index_overlap)
{
  if (!cloud || cloud->empty()) {
    cluster_indices.clear();
    return;
  }

  const bool use_mid_band = mid_threshold > close_threshold;

  pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr mid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> near_map;
  std::vector<int> mid_map;
  std::vector<int> far_map;

  near_cloud->reserve(cloud->size());
  mid_cloud->reserve(cloud->size());
  far_cloud->reserve(cloud->size());
  near_map.reserve(cloud->size());
  mid_map.reserve(cloud->size());
  far_map.reserve(cloud->size());

  auto finalizeCloud = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr & c) {
    c->width = c->points.size();
    c->height = 1;
    c->is_dense = false;
  };

  if (!use_mid_band) {
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      const auto & pt = cloud->points[i];
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
        continue;
      }
      const double r_xy = std::hypot(static_cast<double>(pt.x), static_cast<double>(pt.y));
      if (r_xy < close_threshold) {
        near_cloud->points.push_back(pt);
        near_map.push_back(static_cast<int>(i));
      } else {
        far_cloud->points.push_back(pt);
        far_map.push_back(static_cast<int>(i));
      }
    }
    finalizeCloud(near_cloud);
    finalizeCloud(mid_cloud);
    finalizeCloud(far_cloud);

    cluster_indices.clear();
    appendEuclideanClustersMapped(
      near_cloud, near_map, base_cluster_tolerance * near_tolerance_mult, minClusterSize, maxClusterSize,
      cluster_indices);
    appendEuclideanClustersMapped(
      far_cloud, far_map, base_cluster_tolerance * far_tolerance_mult, minClusterSize, maxClusterSize,
      cluster_indices);
    return;
  }

  const double mid_min = (band_near_mid_overlap_m > 0.0)
    ? std::max(0.0, close_threshold - band_near_mid_overlap_m)
    : close_threshold;
  const double far_min = (band_mid_far_overlap_m > 0.0)
    ? std::max(0.0, mid_threshold - band_mid_far_overlap_m)
    : mid_threshold;

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto & pt = cloud->points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
      continue;
    }
    const double r_xy = std::hypot(static_cast<double>(pt.x), static_cast<double>(pt.y));

    if (r_xy < close_threshold) {
      near_cloud->points.push_back(pt);
      near_map.push_back(static_cast<int>(i));
    }
    if (r_xy >= mid_min && r_xy < mid_threshold) {
      mid_cloud->points.push_back(pt);
      mid_map.push_back(static_cast<int>(i));
    }
    if (r_xy >= far_min) {
      far_cloud->points.push_back(pt);
      far_map.push_back(static_cast<int>(i));
    }
  }

  finalizeCloud(near_cloud);
  finalizeCloud(mid_cloud);
  finalizeCloud(far_cloud);

  cluster_indices.clear();

  appendEuclideanClustersMapped(
    near_cloud, near_map, base_cluster_tolerance * near_tolerance_mult, minClusterSize, maxClusterSize,
    cluster_indices);
  appendEuclideanClustersMapped(
    mid_cloud, mid_map, base_cluster_tolerance * mid_tolerance_mult, minClusterSize, maxClusterSize,
    cluster_indices);
  appendEuclideanClustersMapped(
    far_cloud, far_map, base_cluster_tolerance * far_tolerance_mult, minClusterSize, maxClusterSize,
    cluster_indices);

  const bool bands_overlap =
    (band_near_mid_overlap_m > 0.0 && mid_min < close_threshold) ||
    (band_mid_far_overlap_m > 0.0 && far_min < mid_threshold);
  if (bands_overlap && band_merge_min_index_overlap > 0.0) {
    constexpr double kBandMergeMaxCentroidXyDist = 0.6;
    constexpr double kBandMergeMinZOverlap = 0.5;
    mergeClustersBySortedIndexOverlap(
      cluster_indices,
      band_merge_min_index_overlap,
      cloud,
      kBandMergeMaxCentroidXyDist,
      kBandMergeMinZOverlap);
  }
}

void assignClusterColors(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::vector<pcl::PointIndices> & cluster_indices,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & clustered_cloud)
{
  if (cloud->empty() || cluster_indices.empty()) return;

  clustered_cloud->clear();
  clustered_cloud->points.reserve(cloud->size());

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);

  for (const auto & indices : cluster_indices) {
    int r = dis(gen);
    int g = dis(gen);
    int b = dis(gen);
    for (const auto & index : indices.indices) {
      pcl::PointXYZRGB point;
      point.x = cloud->points[index].x;
      point.y = cloud->points[index].y;
      point.z = cloud->points[index].z;
      point.r = r;
      point.g = g;
      point.b = b;
      clustered_cloud->points.push_back(point);
    }
  }
  clustered_cloud->width = clustered_cloud->points.size();
  clustered_cloud->height = 1;
  clustered_cloud->is_dense = true;
  clustered_cloud->header = cloud->header;
}

void mergeClusters(
  std::vector<pcl::PointIndices> & cluster_indices,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::vector<ClusterStats> & stats,
  double mergeTolerance)
{
  if (cloud->empty() || cluster_indices.empty() || stats.size() != cluster_indices.size()) return;

  std::vector<bool> merged(cluster_indices.size(), false);
  std::vector<ClusterStats> working_stats = stats;

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (merged[i]) continue;

    for (size_t j = i + 1; j < cluster_indices.size(); ++j) {
      if (merged[j]) continue;

      float gap_x = std::max(
        0.f,
        std::max(working_stats[i].min_x, working_stats[j].min_x) -
          std::min(working_stats[i].max_x, working_stats[j].max_x));
      float gap_y = std::max(
        0.f,
        std::max(working_stats[i].min_y, working_stats[j].min_y) -
          std::min(working_stats[i].max_y, working_stats[j].max_y));
      float gap_z = std::max(
        0.f,
        std::max(working_stats[i].min_z, working_stats[j].min_z) -
          std::min(working_stats[i].max_z, working_stats[j].max_z));
      double distance = std::sqrt(gap_x * gap_x + gap_y * gap_y + gap_z * gap_z);

      if (distance < mergeTolerance) {
        std::vector<int> merged_indices = cluster_indices[i].indices;
        merged_indices.insert(
          merged_indices.end(), cluster_indices[j].indices.begin(), cluster_indices[j].indices.end());
        cluster_indices[i].indices = std::move(merged_indices);
        merged[j] = true;
        working_stats[i] = cluster_box::computeSingleClusterStats(cloud, cluster_indices[i]);
      }
    }
  }

  std::vector<pcl::PointIndices> filtered_clusters;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (!merged[i]) {
      filtered_clusters.push_back(cluster_indices[i]);
    }
  }
  cluster_indices = filtered_clusters;
}

namespace
{
bool pointInRectClosed(const cv::Point2d & uv, const cv::Rect2d & r)
{
  if (r.width <= 0.0 || r.height <= 0.0) return false;
  const double x1 = r.x + r.width;
  const double y1 = r.y + r.height;
  return uv.x >= r.x && uv.x <= x1 && uv.y >= r.y && uv.y <= y1;
}

cv::Rect2d expandRectToImageBounds(const cv::Rect2d & r, double expand_fraction, double iw, double ih)
{
  if (r.width <= 0.0 || r.height <= 0.0) return cv::Rect2d();
  if (expand_fraction <= 0.0) {
    const double x0 = std::max(0.0, r.x);
    const double y0 = std::max(0.0, r.y);
    const double x1 = std::min(iw, r.x + r.width);
    const double y1 = std::min(ih, r.y + r.height);
    if (x1 <= x0 || y1 <= y0) return cv::Rect2d();
    return cv::Rect2d(x0, y0, x1 - x0, y1 - y0);
  }
  const double cx = r.x + r.width / 2.0;
  const double cy = r.y + r.height / 2.0;
  const double new_w = r.width * (1.0 + expand_fraction);
  const double new_h = r.height * (1.0 + expand_fraction);
  const double x0 = std::max(0.0, cx - new_w / 2.0);
  const double y0 = std::max(0.0, cy - new_h / 2.0);
  const double x1 = std::min(iw, cx + new_w / 2.0);
  const double y1 = std::min(ih, cy + new_h / 2.0);
  if (x1 <= x0 || y1 <= y0) return cv::Rect2d();
  return cv::Rect2d(x0, y0, x1 - x0, y1 - y0);
}

int qualityTieredMinPoints(const ClusterStats & s, const ProjectionUtilsParams & q)
{
  float width_x = s.max_x - s.min_x;
  float width_y = s.max_y - s.min_y;
  float height = s.max_z - s.min_z;
  float volume = width_x * width_y * height;
  double distance =
    std::sqrt(s.centroid.x() * s.centroid.x() + s.centroid.y() * s.centroid.y() + s.centroid.z() * s.centroid.z());

  int min_points_threshold = q.quality_min_points_default;
  if (distance > q.quality_distance_threshold_far) {
    min_points_threshold = q.quality_min_points_far;
  } else if (distance > q.quality_distance_threshold_medium) {
    min_points_threshold = q.quality_min_points_medium;
  } else if (volume > q.quality_volume_threshold_large) {
    min_points_threshold = q.quality_min_points_large;
  }
  return min_points_threshold;
}

std::vector<pcl::PointIndices> euclideanClustersOnSubset(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::vector<int> & global_indices,
  double cluster_tolerance,
  int min_cluster_size,
  int max_cluster_size)
{
  std::vector<pcl::PointIndices> out;
  if (!cloud || global_indices.size() < 2u) {
    return out;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> to_global;
  dense_cloud->reserve(global_indices.size());
  to_global.reserve(global_indices.size());
  for (int gi : global_indices) {
    if (gi < 0 || static_cast<size_t>(gi) >= cloud->points.size()) continue;
    const auto & pt = cloud->points[static_cast<size_t>(gi)];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    dense_cloud->points.push_back(pt);
    to_global.push_back(gi);
  }
  dense_cloud->width = dense_cloud->points.size();
  dense_cloud->height = 1;
  dense_cloud->is_dense = true;
  if (dense_cloud->size() < 2u) {
    return out;
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(dense_cloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(dense_cloud);
  std::vector<pcl::PointIndices> local_clusters;
  ec.extract(local_clusters);
  out.reserve(local_clusters.size());
  for (const auto & lc : local_clusters) {
    pcl::PointIndices mapped;
    mapped.indices.reserve(lc.indices.size());
    for (int li : lc.indices) {
      mapped.indices.push_back(to_global[static_cast<size_t>(li)]);
    }
    out.push_back(std::move(mapped));
  }
  return out;
}
}  // namespace

std::vector<ClusterCandidate> buildCandidatesFromDetectionRois(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const Eigen::Matrix<double, 3, 4> & lidar_to_image,
  const std::array<double, 12> & projection_matrix,
  const vision_msgs::msg::Detection2DArray & detections,
  float object_detection_confidence,
  double roi_expand_fraction,
  double cluster_tolerance,
  int min_cluster_size,
  int max_cluster_size,
  int image_width,
  int image_height,
  bool claim_points_unique)
{
  constexpr size_t kMaxCandidatesPerDetection = 6;
  std::vector<ClusterCandidate> result;
  if (!cloud || cloud->empty() || detections.detections.empty()) {
    return result;
  }
  const auto & params = getParams();
  const int iw_param = (image_width > 0 && image_height > 0)
                         ? image_width
                         : (params.image_width > 0 ? params.image_width : kDefaultImageWidth);
  const int ih_param = (image_width > 0 && image_height > 0)
                         ? image_height
                         : (params.image_height > 0 ? params.image_height : kDefaultImageHeight);
  const double iw = static_cast<double>(iw_param);
  const double ih = static_cast<double>(ih_param);
  auto assumedObjectHeightMeters = [](const std::string & class_id) -> double {
      const auto & p = getParams();
      if (class_id == "person") return p.assumed_height_person_m;
      if (class_id == "car") return p.assumed_height_car_m;
      if (class_id == "truck") return p.assumed_height_truck_bus_m;
      if (class_id == "bus") return p.assumed_height_bus_m;
      if (class_id == "traffic_sign" || class_id == "stop_sign" || class_id == "traffic_light") {
        return p.assumed_height_traffic_control_m;
      }
      return p.assumed_height_car_m;
    };

  std::vector<int> claim_count(cloud->points.size(), 0);

  std::vector<int> order(detections.detections.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    const double sa =
      detections.detections[static_cast<size_t>(a)].results.empty()
        ? 0.0
        : static_cast<double>(detections.detections[static_cast<size_t>(a)].results[0].hypothesis.score);
    const double sb =
      detections.detections[static_cast<size_t>(b)].results.empty()
        ? 0.0
        : static_cast<double>(detections.detections[static_cast<size_t>(b)].results[0].hypothesis.score);
    return sa > sb;
  });

  for (int det_idx : order) {
    const auto & det = detections.detections[static_cast<size_t>(det_idx)];
    if (!det.results.empty() && det.results[0].hypothesis.score < object_detection_confidence) {
      continue;
    }
    const auto & b = det.bbox;
    const std::string cls = det.results.empty() ? std::string() : det.results[0].hypothesis.class_id;
    const double assumed_height_m = assumedObjectHeightMeters(cls);
    const double focal_y = std::max(1e-6, std::abs(projection_matrix[5]));
    const cv::Rect2d det_rect(
      b.center.position.x - b.size_x / 2.0, b.center.position.y - b.size_y / 2.0, b.size_x, b.size_y);
    const cv::Rect2d roi_rect = expandRectToImageBounds(det_rect, roi_expand_fraction, iw, ih);
    if (roi_rect.width <= 0.0 || roi_rect.height <= 0.0) {
      continue;
    }
    const double expected_depth = (focal_y * assumed_height_m) / std::max(1.0, det_rect.height);
    const bool is_heavy_vehicle = (cls == "truck" || cls == "bus");
    const double depth_band_low_mult = is_heavy_vehicle ? 0.25 : ((expected_depth < 15.0) ? 0.50 : 0.35);
    const double depth_band_high_mult = is_heavy_vehicle ? 3.5 : ((expected_depth < 15.0) ? 1.8 : 2.5);
    const double depth_min = expected_depth * depth_band_low_mult;
    const double depth_max = expected_depth * depth_band_high_mult;

    std::vector<int> roi_globals;
    roi_globals.reserve(cloud->size() / 8u + 8u);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      const auto & pt = cloud->points[i];
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      auto uv = projectLidarToCamera(lidar_to_image, pt);
      if (!uv) continue;
      if (uv->x < 0.0 || uv->x >= iw || uv->y < 0.0 || uv->y >= ih) continue;
      if (!pointInRectClosed(*uv, roi_rect)) continue;
      const Eigen::Vector4d lidar_pt(pt.x, pt.y, pt.z, 1.0);
      const Eigen::Vector3d projected = lidar_to_image * lidar_pt;
      const double cam_depth = projected.z();
      if (cam_depth < depth_min || cam_depth > depth_max) continue;
      roi_globals.push_back(static_cast<int>(i));
    }

    if (static_cast<int>(roi_globals.size()) < min_cluster_size) {
      continue;
    }

    std::vector<pcl::PointIndices> clusters =
      euclideanClustersOnSubset(cloud, roi_globals, cluster_tolerance, min_cluster_size, max_cluster_size);
    if (clusters.empty()) {
      continue;
    }

    struct ScoredCluster
    {
      size_t k;
      double score;
    };
    std::vector<ScoredCluster> scored;
    scored.reserve(clusters.size());
    for (size_t k = 0; k < clusters.size(); ++k) {
      int inside = 0;
      int proj_count = 0;
      double u_min = std::numeric_limits<double>::infinity();
      double v_min = std::numeric_limits<double>::infinity();
      double u_max = -std::numeric_limits<double>::infinity();
      double v_max = -std::numeric_limits<double>::infinity();
      for (int gi : clusters[k].indices) {
        const auto & pt = cloud->points[static_cast<size_t>(gi)];
        auto uv = projectLidarToCamera(lidar_to_image, pt);
        if (!uv) continue;
        if (uv->x < 0.0 || uv->x >= iw || uv->y < 0.0 || uv->y >= ih) continue;
        ++proj_count;
        u_min = std::min(u_min, uv->x);
        v_min = std::min(v_min, uv->y);
        u_max = std::max(u_max, uv->x);
        v_max = std::max(v_max, uv->y);
        if (pointInRectClosed(*uv, det_rect)) ++inside;
      }

      if (inside <= 0 || proj_count <= 0) {
        continue;
      }

      const double proj_w = std::max(1e-6, u_max - u_min);
      const double proj_h = std::max(1e-6, v_max - v_min);
      const double det_w = std::max(1e-6, det_rect.width);
      const double det_h = std::max(1e-6, det_rect.height);
      const double w_ratio = std::min(proj_w / det_w, det_w / proj_w);
      const double h_ratio = std::min(proj_h / det_h, det_h / proj_h);
      const double size_match = std::max(0.0, std::min(1.0, w_ratio * h_ratio));

      const cv::Rect2d proj_rect(u_min, v_min, proj_w, proj_h);
      const cv::Rect2d inter = proj_rect & det_rect;
      const double inter_area = (inter.width > 0.0 && inter.height > 0.0) ? inter.area() : 0.0;
      const double det_area = std::max(1e-6, det_rect.area());
      const double det_coverage = std::max(0.0, std::min(1.0, inter_area / det_area));

      const ClusterStats cand_stats = cluster_box::computeSingleClusterStats(cloud, clusters[k]);
      const double cluster_range = std::hypot(
        static_cast<double>(cand_stats.centroid.x()), static_cast<double>(cand_stats.centroid.y()));
      const double depth_ratio = cluster_range / std::max(0.5, expected_depth);
      double depth_score = 0.0;
      if (depth_ratio >= 0.5 && depth_ratio <= 2.0) {
        depth_score = 1.0 - std::abs(depth_ratio - 1.0) * 0.8;
        depth_score = std::max(0.0, depth_score);
      }

      const double inside_score =
        static_cast<double>(inside) / std::max(1.0, static_cast<double>(proj_count));
      int reused_points = 0;
      if (claim_points_unique) {
        for (int gi : clusters[k].indices) {
          if (gi >= 0 && static_cast<size_t>(gi) < claim_count.size() &&
              claim_count[static_cast<size_t>(gi)] > 0)
          {
            ++reused_points;
          }
        }
      }
      const double reuse_penalty = claim_points_unique
                                     ? static_cast<double>(reused_points) /
                                       std::max(1.0, static_cast<double>(clusters[k].indices.size()))
                                     : 0.0;
      const double base_score =
        0.30 * inside_score + 0.36 * size_match + 0.24 * det_coverage + 0.10 * depth_score;
      const double reuse_penalty_scale =
        std::max(0.0, std::min(1.0, params.association_roi_soft_claim_penalty_scale));
      const double score = base_score * (1.0 - reuse_penalty_scale * reuse_penalty);
      if (score <= 0.0 || static_cast<int>(clusters[k].indices.size()) < min_cluster_size) {
        continue;
      }
      scored.push_back({k, score});
    }
    std::sort(
      scored.begin(), scored.end(),
      [](const ScoredCluster & a, const ScoredCluster & b) { return a.score > b.score; });
    if (scored.size() > kMaxCandidatesPerDetection) {
      scored.resize(kMaxCandidatesPerDetection);
    }
    for (const auto & sc : scored) {
      ClusterCandidate cand;
      cand.indices = std::move(clusters[sc.k]);
      cand.stats = cluster_box::computeSingleClusterStats(cloud, cand.indices);
      // ROI seeding carries det_idx/expected_depth for downstream gating, but this is not
      // a real IoU match yet (real matching happens in assignCandidatesToDetectionsByIOU()).
      cand.match = ClusterDetectionMatch{0, det_idx, 0.0, -1.0, expected_depth};
      cand.roi_seed_det_idx = det_idx;
      cand.roi_seed_score = sc.score;
      if (claim_points_unique) {
        for (int gi : cand.indices.indices) {
          if (gi >= 0 && static_cast<size_t>(gi) < claim_count.size()) {
            ++claim_count[static_cast<size_t>(gi)];
          }
        }
      }
      result.push_back(std::move(cand));
    }
  }
  return result;
}

void filterCandidatesByClassAwareConstraints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  std::vector<ClusterCandidate> & candidates,
  const vision_msgs::msg::Detection2DArray & detections,
  bool use_tiered_min_points)
{
  constexpr float kMinDimensionForAspect = 0.05f;
  constexpr float kMinDimForDensity = 0.15f;
  constexpr float kMinVolumeForDensity = 0.1f;
  constexpr float kMaxVehicleLength = 20.0f;

  const auto & q = getParams();

  std::vector<ClusterCandidate> kept;
  kept.reserve(candidates.size());
  for (auto & cand : candidates) {
    const auto & s = cand.stats;

    float width_x = s.max_x - s.min_x;
    float width_y = s.max_y - s.min_y;
    float height = s.max_z - s.min_z;
    float volume = width_x * width_y * height;
    double distance =
      std::sqrt(s.centroid.x() * s.centroid.x() + s.centroid.y() * s.centroid.y() + s.centroid.z() * s.centroid.z());

    if (s.num_points < q.quality_min_points || height < q.quality_min_height) continue;

    int min_points_threshold = use_tiered_min_points ? qualityTieredMinPoints(s, q) : q.quality_min_points;
    if (s.num_points < min_points_threshold) continue;

    const float min_dim = std::min({width_x, width_y, height});
    const bool thin_object = min_dim < kMinDimForDensity;
    if (!thin_object && volume > kMinVolumeForDensity) {
      // Exempt sparse, small-volume clusters at range from min-density rejection.
      bool exempt_by_range = false;
      if (distance > q.quality_distance_threshold_medium && volume < 2.0f) {
        exempt_by_range = true;
      }

      if (volume <= q.quality_density_lower_bound_max_volume && !exempt_by_range) {
        float density = s.num_points / volume;
        if (density < q.quality_min_density) continue;
        if (density > q.quality_max_density) continue;
      } else if (!exempt_by_range) {
        float density = s.num_points / volume;
        if (density > q.quality_max_density) continue;
      }
    }

    float max_dim = std::max({width_x, width_y, height});
    if (max_dim > q.quality_max_dimension) continue;
    if (min_dim > kMinDimensionForAspect && max_dim / min_dim > q.quality_max_aspect_ratio) continue;

    if (distance > q.quality_max_distance) continue;

    if (cand.match.has_value()) {
      if (cand.match->expected_depth > 0.0) {
        const double cluster_range = std::hypot(static_cast<double>(s.centroid.x()), static_cast<double>(s.centroid.y()));
        const double depth_ratio = cluster_range / cand.match->expected_depth;
        if (depth_ratio < 0.3 || depth_ratio > 3.0) continue;
      }
      const int di = cand.match->det_idx;
      if (di >= 0 && static_cast<size_t>(di) < detections.detections.size()) {
        const auto & det = detections.detections[static_cast<size_t>(di)];
        const std::string cid = det.results.empty() ? std::string() : det.results[0].hypothesis.class_id;
        const bool is_person = cid == "person";
        const bool is_vehicle = cid == "car" || cid == "truck" || cid == "bus";
        const bool is_sign = cid == "traffic_sign" || cid == "stop_sign" || cid == "traffic_light";
        float cap_h = q.quality_max_dimension;
        float cap_xy = q.quality_max_dimension;
        if (is_person) {
          cap_h = std::min(cap_h, q.quality_person_max_height_m);
          cap_xy = std::min(cap_xy, q.quality_person_max_footprint_xy_m);
        } else if (is_vehicle) {
          cap_h = std::min(cap_h, q.quality_vehicle_max_height_m);
          cap_xy = std::min(cap_xy, q.quality_vehicle_max_footprint_xy_m);
        } else if (is_sign) {
          cap_h = std::min(cap_h, q.quality_sign_max_height_m);
          cap_xy = std::min(cap_xy, q.quality_sign_max_footprint_xy_m);
        }
        if (height > cap_h) continue;
        if (is_person || is_vehicle || is_sign) {
          const float max_foot = std::max(width_x, width_y);
          if (is_vehicle) {
            // Use oriented box dimensions for vehicles to avoid yaw-inflated AABB footprint rejections.
            const Box3D box = cluster_box::computeClusterBox(cloud, cand.indices);
            const float veh_length = std::max(box.size.x(), box.size.y());
            const float veh_width = std::min(box.size.x(), box.size.y());
            if (veh_width > cap_xy) continue;
            if (veh_length > kMaxVehicleLength) continue;
          } else if (max_foot > cap_xy) {
            continue;
          }
        }
      }
    }

    kept.push_back(std::move(cand));
  }
  candidates = std::move(kept);
}

void deduplicateCandidatesBySharedPoints(
  std::vector<ClusterCandidate> & candidates,
  double min_shared_ratio,
  const vision_msgs::msg::Detection2DArray * detections)
{
  if (candidates.size() < 2u) {
    return;
  }
  const double thresh = std::max(0.0, std::min(1.0, min_shared_ratio));
  if (thresh <= 0.0) {
    return;
  }

  std::vector<std::vector<int>> sorted_indices(candidates.size());
  for (size_t i = 0; i < candidates.size(); ++i) {
    sorted_indices[i] = candidates[i].indices.indices;
    std::sort(sorted_indices[i].begin(), sorted_indices[i].end());
    sorted_indices[i].erase(std::unique(sorted_indices[i].begin(), sorted_indices[i].end()), sorted_indices[i].end());
  }

  auto overlap_ratio = [](const std::vector<int> & a, const std::vector<int> & b) -> double {
      if (a.empty() || b.empty()) return 0.0;
      size_t inter = 0;
      size_t ia = 0;
      size_t ib = 0;
      while (ia < a.size() && ib < b.size()) {
        if (a[ia] == b[ib]) {
          ++inter;
          ++ia;
          ++ib;
        } else if (a[ia] < b[ib]) {
          ++ia;
        } else {
          ++ib;
        }
      }
      const size_t denom = std::min(a.size(), b.size());
      return denom == 0u ? 0.0 : static_cast<double>(inter) / static_cast<double>(denom);
    };

  std::vector<char> keep(candidates.size(), 1);
  auto candidateClassId = [&](const ClusterCandidate & c) -> std::string {
      if (!detections || !c.match.has_value()) {
        return std::string();
      }
      const int di = c.match->det_idx;
      if (di < 0 || static_cast<size_t>(di) >= detections->detections.size()) {
        return std::string();
      }
      const auto & d = detections->detections[static_cast<size_t>(di)];
      return d.results.empty() ? std::string() : d.results[0].hypothesis.class_id;
    };
  auto classAwareMinSeparationForDedup = [](const std::string & ci, const std::string & cj) -> double {
      const bool i_person = ci == "person";
      const bool j_person = cj == "person";
      if (i_person && j_person) {
        return 0.35;
      }
      if (i_person || j_person) {
        return 0.50;
      }
      return 0.80;
    };
  auto candidate_score = [](const ClusterCandidate & c) -> double {
      // In ROI-seeded pre-association mode, `match->iou` is placeholder-only and should not
      // drive dedup ranking. Prefer the ROI candidate ranking score when available.
      if (c.roi_seed_det_idx.has_value()) {
        if (c.roi_seed_score >= 0.0) {
          return c.roi_seed_score;
        }
        // Backward-safe fallback for candidates created before roi_seed_score was set.
        return static_cast<double>(std::max(0, c.stats.num_points));
      }
      if (!c.match.has_value()) {
        return 0.0;
      }
      return c.match->iou;
    };

  for (size_t i = 0; i < candidates.size(); ++i) {
    if (!keep[i]) continue;
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (!keep[j]) continue;
      if (overlap_ratio(sorted_indices[i], sorted_indices[j]) <= thresh) continue;

      // Preserve nearby distinct objects that may share ROI-extracted points.
      const bool both_matched = candidates[i].match.has_value() && candidates[j].match.has_value();
      if (both_matched && candidates[i].match->det_idx != candidates[j].match->det_idx) {
        const double cx_i = candidates[i].stats.centroid.x();
        const double cy_i = candidates[i].stats.centroid.y();
        const double cx_j = candidates[j].stats.centroid.x();
        const double cy_j = candidates[j].stats.centroid.y();
        const double sep = std::hypot(cx_i - cx_j, cy_i - cy_j);
        const std::string ci = candidateClassId(candidates[i]);
        const std::string cj = candidateClassId(candidates[j]);
        const double min_sep_for_dedup = classAwareMinSeparationForDedup(ci, cj);
        if (sep > min_sep_for_dedup) {
          continue;
        }
      }

      const double si = candidate_score(candidates[i]);
      const double sj = candidate_score(candidates[j]);
      if (sj > si) {
        keep[i] = 0;
        break;
      }
      keep[j] = 0;
    }
  }

  std::vector<ClusterCandidate> deduped;
  deduped.reserve(candidates.size());
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (keep[i]) {
      deduped.push_back(std::move(candidates[i]));
    }
  }
  candidates = std::move(deduped);
}

bool computeClusterCentroid(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const pcl::PointIndices & cluster_indices,
  pcl::PointXYZ & centroid)
{
  if (cloud->empty() || cluster_indices.indices.empty()) return false;

  Eigen::Vector4f centroid_eigen;
  pcl::compute3DCentroid(*cloud, cluster_indices, centroid_eigen);

  centroid.x = centroid_eigen[0];
  centroid.y = centroid_eigen[1];
  centroid.z = centroid_eigen[2];

  return true;
}

namespace
{
std::optional<cv::Rect2d> percentileRectFromProjectedPoints(
  const std::vector<cv::Point2d> & pts, double low_q, double high_q)
{
  if (pts.empty() || low_q < 0.0 || high_q > 1.0 || low_q >= high_q) {
    return std::nullopt;
  }
  std::vector<double> us;
  std::vector<double> vs;
  us.reserve(pts.size());
  vs.reserve(pts.size());
  for (const auto & p : pts) {
    us.push_back(p.x);
    vs.push_back(p.y);
  }
  std::sort(us.begin(), us.end());
  std::sort(vs.begin(), vs.end());
  const size_t n = us.size();
  const size_t i0 = std::min(n - 1, static_cast<size_t>(std::floor(low_q * static_cast<double>(n - 1))));
  const size_t i1 = std::min(n - 1, static_cast<size_t>(std::ceil(high_q * static_cast<double>(n - 1))));
  const double u0 = us[i0];
  const double u1 = us[i1];
  const double v0 = vs[i0];
  const double v1 = vs[i1];
  if (u1 <= u0 || v1 <= v0) {
    return std::nullopt;
  }
  return cv::Rect2d(u0, v0, u1 - u0, v1 - v0);
}

std::optional<cv::Rect2d> projectAABBRect(
  const ClusterStats & stats, const Eigen::Matrix<double, 3, 4> & lidar_to_image, double image_w, double image_h)
{
  const float xs[2] = {stats.min_x, stats.max_x};
  const float ys[2] = {stats.min_y, stats.max_y};
  const float zs[2] = {stats.min_z, stats.max_z};

  double u0 = std::numeric_limits<double>::infinity(), v0 = std::numeric_limits<double>::infinity();
  double u1 = -std::numeric_limits<double>::infinity(), v1 = -std::numeric_limits<double>::infinity();
  size_t valid_count = 0;

  for (int xi = 0; xi < 2; ++xi) {
    for (int yi = 0; yi < 2; ++yi) {
      for (int zi = 0; zi < 2; ++zi) {
        pcl::PointXYZ pt;
        pt.x = xs[xi];
        pt.y = ys[yi];
        pt.z = zs[zi];
        auto uv = projectLidarToCamera(lidar_to_image, pt);
        if (!uv) continue;
        u0 = std::min(u0, uv->x);
        v0 = std::min(v0, uv->y);
        u1 = std::max(u1, uv->x);
        v1 = std::max(v1, uv->y);
        ++valid_count;
      }
    }
  }

  if (valid_count == 0 || u1 <= u0 || v1 <= v0) {
    return std::nullopt;
  }
  const double u0_clip = std::max(0.0, u0);
  const double v0_clip = std::max(0.0, v0);
  const double u1_clip = std::min(image_w, u1);
  const double v1_clip = std::min(image_h, v1);
  if (u1_clip <= u0_clip || v1_clip <= v0_clip) {
    return std::nullopt;
  }
  return cv::Rect2d(u0_clip, v0_clip, u1_clip - u0_clip, v1_clip - v0_clip);
}

std::optional<cv::Rect2d> projectOrientedBoxImageRect(
  const Box3D & box,
  const Eigen::Matrix<double, 3, 4> & lidar_to_image,
  double image_w,
  double image_h)
{
  if (box.size.x() <= 1e-6f || box.size.y() <= 1e-6f || box.size.z() <= 1e-6f) {
    return std::nullopt;
  }
  const double c = std::cos(box.yaw);
  const double s = std::sin(box.yaw);
  const double hx = 0.5 * static_cast<double>(box.size.x());
  const double hy = 0.5 * static_cast<double>(box.size.y());
  const double hz = 0.5 * static_cast<double>(box.size.z());
  const double cx = static_cast<double>(box.center.x());
  const double cy = static_cast<double>(box.center.y());
  const double cz = static_cast<double>(box.center.z());

  double u0 = std::numeric_limits<double>::infinity(), v0 = std::numeric_limits<double>::infinity();
  double u1 = -std::numeric_limits<double>::infinity(), v1 = -std::numeric_limits<double>::infinity();
  size_t valid_count = 0;

  for (int ix = -1; ix <= 1; ix += 2) {
    for (int iy = -1; iy <= 1; iy += 2) {
      for (int iz = -1; iz <= 1; iz += 2) {
        const double lx = static_cast<double>(ix) * hx;
        const double ly = static_cast<double>(iy) * hy;
        const double lz = static_cast<double>(iz) * hz;
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(cx + c * lx - s * ly);
        pt.y = static_cast<float>(cy + s * lx + c * ly);
        pt.z = static_cast<float>(cz + lz);
        auto uv = projectLidarToCamera(lidar_to_image, pt);
        if (!uv) continue;
        u0 = std::min(u0, uv->x);
        v0 = std::min(v0, uv->y);
        u1 = std::max(u1, uv->x);
        v1 = std::max(v1, uv->y);
        ++valid_count;
      }
    }
  }

  if (valid_count == 0 || u1 <= u0 || v1 <= v0) {
    return std::nullopt;
  }
  const double u0_clip = std::max(0.0, u0);
  const double v0_clip = std::max(0.0, v0);
  const double u1_clip = std::min(image_w, u1);
  const double v1_clip = std::min(image_h, v1);
  if (u1_clip <= u0_clip || v1_clip <= v0_clip) {
    return std::nullopt;
  }
  return cv::Rect2d(u0_clip, v0_clip, u1_clip - u0_clip, v1_clip - v0_clip);
}
}  // namespace

void assignCandidatesToDetectionsByIOU(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  std::vector<ClusterCandidate> & candidates,
  const vision_msgs::msg::Detection2DArray & detections,
  const geometry_msgs::msg::TransformStamped & transform,
  const std::array<double, 12> & projection_matrix,
  float object_detection_confidence,
  int image_width,
  int image_height)
{
  if (candidates.empty()) return;

  if (detections.detections.empty()) {
    candidates.clear();
    return;
  }

  const Eigen::Matrix<double, 3, 4> lidar_to_image = buildLidarToImageMatrix(transform, projection_matrix);
  const auto & params = getParams();
  const int iw_param = (image_width > 0 && image_height > 0)
                         ? image_width
                         : (params.image_width > 0 ? params.image_width : kDefaultImageWidth);
  const int ih_param = (image_width > 0 && image_height > 0)
                         ? image_height
                         : (params.image_height > 0 ? params.image_height : kDefaultImageHeight);
  const double iw = static_cast<double>(iw_param);
  const double ih = static_cast<double>(ih_param);
  const double min_iou = params.min_iou_threshold;

  // First-pass scoring weights (sum to 1). Assignment uses per-detection argmax for ROI-seeded
  // candidates and global one-to-one Hungarian matching for unseeded/global candidates.
  constexpr double kAssocWIoU = 0.30;
  constexpr double kAssocWInsideFrac = 0.28;
  constexpr double kAssocWAr = 0.18;
  constexpr double kAssocWCentroid = 0.14;
  constexpr double kAssocWPoints = 0.10;

  struct Pair
  {
    size_t cand_idx;
    int det_idx;
    double iou;
    double combined_score;
  };

  std::vector<Pair> pairs;

  auto shrinkInner = [](const cv::Rect2d & det, double frac) -> cv::Rect2d {
    if (det.width <= 0.0 || det.height <= 0.0) return cv::Rect2d();
    const double cx = det.x + det.width * 0.5;
    const double cy = det.y + det.height * 0.5;
    const double nw = det.width * frac;
    const double nh = det.height * frac;
    return cv::Rect2d(cx - nw * 0.5, cy - nh * 0.5, nw, nh);
  };
  auto detectionRectAt = [&](int det_idx) -> std::optional<cv::Rect2d> {
    if (det_idx < 0 || static_cast<size_t>(det_idx) >= detections.detections.size()) {
      return std::nullopt;
    }
    const auto & b = detections.detections[static_cast<size_t>(det_idx)].bbox;
    const cv::Rect2d r(
      b.center.position.x - b.size_x / 2.0, b.center.position.y - b.size_y / 2.0, b.size_x, b.size_y);
    if (r.width <= 0.0 || r.height <= 0.0) {
      return std::nullopt;
    }
    return r;
  };
  auto boxOverlapIoU = [](const cv::Rect2d & a, const cv::Rect2d & b) -> double {
    const cv::Rect2d inter = a & b;
    const double inter_area = (inter.width > 0.0 && inter.height > 0.0) ? inter.area() : 0.0;
    const double uni = a.area() + b.area() - inter_area;
    return (uni > 0.0) ? inter_area / uni : 0.0;
  };
  constexpr double kSeedNeighborDetIouThreshold = 0.35;
  constexpr double kSeedNeighborMatchPenalty = 0.10;

  if (params.association_strict_matching) {
    constexpr double kAssocPointRectLowQ = 0.05;
    constexpr double kAssocPointRectHighQ = 0.95;
    struct CandGeom
    {
      std::vector<cv::Point2d> uvs;
      bool centroid_ok{false};
      cv::Point2d centroid_uv{0.0, 0.0};
      std::optional<cv::Rect2d> point_rect;
      std::optional<cv::Rect2d> oriented_rect;
      std::optional<cv::Rect2d> aabb_rect;
    };
    std::vector<CandGeom> geoms(candidates.size());
    for (size_t c = 0; c < candidates.size(); ++c) {
      CandGeom & g = geoms[c];
      g.aabb_rect = projectAABBRect(candidates[c].stats, lidar_to_image, iw, ih);
      if (cloud && !candidates[c].indices.indices.empty()) {
        const Box3D obox = cluster_box::computeClusterBox(cloud, candidates[c].indices);
        g.oriented_rect = projectOrientedBoxImageRect(obox, lidar_to_image, iw, ih);
      }
      pcl::PointXYZ centroid_pt;
      centroid_pt.x = candidates[c].stats.centroid.x();
      centroid_pt.y = candidates[c].stats.centroid.y();
      centroid_pt.z = candidates[c].stats.centroid.z();
      auto cuv = projectLidarToCamera(lidar_to_image, centroid_pt);
      if (cuv && cuv->x >= 0.0 && cuv->x < iw && cuv->y >= 0.0 && cuv->y < ih) {
        g.centroid_ok = true;
        g.centroid_uv = *cuv;
      }
      if (!cloud) continue;
      double u0 = std::numeric_limits<double>::infinity();
      double v0 = std::numeric_limits<double>::infinity();
      double u1 = -std::numeric_limits<double>::infinity();
      double v1 = -std::numeric_limits<double>::infinity();
      for (int idx : candidates[c].indices.indices) {
        if (idx < 0 || static_cast<size_t>(idx) >= cloud->points.size()) continue;
        const auto & pt = cloud->points[static_cast<size_t>(idx)];
        auto uv = projectLidarToCamera(lidar_to_image, pt);
        if (!uv) continue;
        if (uv->x < 0.0 || uv->x >= iw || uv->y < 0.0 || uv->y >= ih) continue;
        g.uvs.push_back(*uv);
        u0 = std::min(u0, uv->x);
        v0 = std::min(v0, uv->y);
        u1 = std::max(u1, uv->x);
        v1 = std::max(v1, uv->y);
      }
      if (g.uvs.size() >= 2u && u1 > u0 && v1 > v0) {
        g.point_rect = percentileRectFromProjectedPoints(g.uvs, kAssocPointRectLowQ, kAssocPointRectHighQ);
      } else if (g.uvs.size() == 1u) {
        g.point_rect = cv::Rect2d(g.uvs[0].x - 3.0, g.uvs[0].y - 3.0, 6.0, 6.0);
      }
    }

    for (size_t c = 0; c < candidates.size(); ++c) {
      const CandGeom & g = geoms[c];
      const auto & st = candidates[c].stats;
      const int seeded_det_idx = candidates[c].roi_seed_det_idx.value_or(-1);
      const std::optional<cv::Rect2d> seeded_det_rect =
        (seeded_det_idx >= 0) ? detectionRectAt(seeded_det_idx) : std::nullopt;
      for (int d = 0; d < static_cast<int>(detections.detections.size()); ++d) {
        if (seeded_det_idx >= 0 && d != seeded_det_idx) {
          const std::optional<cv::Rect2d> det_rect_for_seed_gate = detectionRectAt(d);
          if (!seeded_det_rect || !det_rect_for_seed_gate ||
            boxOverlapIoU(*seeded_det_rect, *det_rect_for_seed_gate) < kSeedNeighborDetIouThreshold)
          {
            continue;
          }
        }
        const auto & det = detections.detections[static_cast<size_t>(d)];
        if (!det.results.empty() && det.results[0].hypothesis.score < object_detection_confidence) {
          continue;
        }
        const auto & b = det.bbox;
        const cv::Rect2d det_rect(
          b.center.position.x - b.size_x / 2.0, b.center.position.y - b.size_y / 2.0, b.size_x, b.size_y);
        const cv::Rect2d inner = shrinkInner(det_rect, params.association_centroid_inner_box_fraction);
        if (inner.width <= 0.0 || inner.height <= 0.0) continue;
        if (!g.centroid_ok) continue;

        std::optional<cv::Rect2d> cand_rect;
        if (params.association_use_point_projection_rect_for_iou && g.point_rect) {
          cand_rect = g.point_rect;
        } else if (g.oriented_rect) {
          cand_rect = g.oriented_rect;
        } else {
          cand_rect = g.aabb_rect;
        }
        if (!cand_rect || cand_rect->width <= 0.0 || cand_rect->height <= 0.0) continue;

        const cv::Rect2d inter = *cand_rect & det_rect;
        const double inter_area = (inter.width > 0.0 && inter.height > 0.0) ? inter.area() : 0.0;
        const double uni = cand_rect->area() + det_rect.area() - inter_area;
        if (uni <= 0.0) continue;
        const double iou = inter_area / uni;
        if (iou < min_iou) continue;

        if (g.uvs.empty()) continue;
        size_t inside = 0;
        for (const auto & uv : g.uvs) {
          if (pointInRectClosed(uv, det_rect)) ++inside;
        }

        if (st.num_points < params.quality_min_points) continue;
        const bool is_roi_seeded_candidate = candidates[c].roi_seed_det_idx.has_value();
        if (!is_roi_seeded_candidate && st.num_points < qualityTieredMinPoints(st, params)) continue;

        const double cand_ar = cand_rect->width / std::max(1e-6, cand_rect->height);
        const double det_ar = b.size_x / std::max(1e-6, b.size_y);
        const double rr = cand_ar / std::max(1e-9, det_ar);
        const double ar_score = std::min(rr, 1.0 / rr);
        const double actual_inside_frac =
          static_cast<double>(inside) / std::max(1.0, static_cast<double>(g.uvs.size()));
        const double det_cx = b.center.position.x;
        const double det_cy = b.center.position.y;
        const double dist_c = std::hypot(g.centroid_uv.x - det_cx, g.centroid_uv.y - det_cy);
        const double det_scale = std::max(1e-3, 0.35 * std::hypot(b.size_x, b.size_y));
        const double centroid_score = std::exp(-dist_c / det_scale);
        const double point_score =
          std::min(1.0, std::log(1.0 + static_cast<double>(st.num_points)) / std::log(1.0 + 120.0));

        if (!pointInRectClosed(g.centroid_uv, inner)) continue;
        if (actual_inside_frac < params.association_min_inside_point_fraction) continue;
        if (ar_score < params.association_min_ar_consistency_score) continue;
        const double det_coverage = inter_area / std::max(1e-6, det_rect.area());
        if (det_coverage < 0.30) continue;

        const bool is_seed_neighbor_match = (seeded_det_idx >= 0 && d != seeded_det_idx);
        const double combined_score_raw = kAssocWIoU * iou + kAssocWInsideFrac * actual_inside_frac +
          kAssocWAr * ar_score + kAssocWCentroid * centroid_score + kAssocWPoints * point_score;
        const double combined_score =
          combined_score_raw - (is_seed_neighbor_match ? kSeedNeighborMatchPenalty : 0.0);
        if (combined_score < params.association_min_combined_score_strict) continue;

        pairs.push_back({c, d, iou, combined_score});
      }
    }
  } else {
    for (size_t c = 0; c < candidates.size(); ++c) {
      const int seeded_det_idx = candidates[c].roi_seed_det_idx.value_or(-1);
      const std::optional<cv::Rect2d> seeded_det_rect =
        (seeded_det_idx >= 0) ? detectionRectAt(seeded_det_idx) : std::nullopt;
      auto cluster_rect = projectAABBRect(candidates[c].stats, lidar_to_image, iw, ih);

      if (cluster_rect) {
        for (size_t d = 0; d < detections.detections.size(); ++d) {
          if (seeded_det_idx >= 0 && static_cast<int>(d) != seeded_det_idx) {
            const std::optional<cv::Rect2d> det_rect_for_seed_gate = detectionRectAt(static_cast<int>(d));
            if (!seeded_det_rect || !det_rect_for_seed_gate ||
              boxOverlapIoU(*seeded_det_rect, *det_rect_for_seed_gate) < kSeedNeighborDetIouThreshold)
            {
              continue;
            }
          }
          const auto & det = detections.detections[d];
          if (!det.results.empty() && det.results[0].hypothesis.score < object_detection_confidence) {
            continue;
          }
          const auto & b = det.bbox;
          const cv::Rect2d det_rect(
            b.center.position.x - b.size_x / 2.0, b.center.position.y - b.size_y / 2.0, b.size_x, b.size_y);
          const cv::Rect2d inter = *cluster_rect & det_rect;
          const double inter_area = (inter.width > 0 && inter.height > 0) ? inter.area() : 0.0;
          const double uni = cluster_rect->area() + det_rect.area() - inter_area;
          if (uni > 0.0) {
            const double iou = inter_area / uni;
            if (iou >= min_iou) {
              const bool is_seed_neighbor_match = (seeded_det_idx >= 0 && static_cast<int>(d) != seeded_det_idx);
              const double combined_score =
                iou - (is_seed_neighbor_match ? kSeedNeighborMatchPenalty : 0.0);
              pairs.push_back({c, static_cast<int>(d), iou, combined_score});
            }
          }
        }
      } else if (params.association_allow_aabb_centroid_fallback) {
        pcl::PointXYZ centroid_pt;
        centroid_pt.x = candidates[c].stats.centroid.x();
        centroid_pt.y = candidates[c].stats.centroid.y();
        centroid_pt.z = candidates[c].stats.centroid.z();
        auto uv = projectLidarToCamera(lidar_to_image, centroid_pt);
        if (uv && uv->x >= 0 && uv->x < iw && uv->y >= 0 && uv->y < ih) {
          for (size_t d = 0; d < detections.detections.size(); ++d) {
            if (seeded_det_idx >= 0 && static_cast<int>(d) != seeded_det_idx) {
              const std::optional<cv::Rect2d> det_rect_for_seed_gate = detectionRectAt(static_cast<int>(d));
              if (!seeded_det_rect || !det_rect_for_seed_gate ||
                boxOverlapIoU(*seeded_det_rect, *det_rect_for_seed_gate) < kSeedNeighborDetIouThreshold)
              {
                continue;
              }
            }
            const auto & det = detections.detections[d];
            if (!det.results.empty() && det.results[0].hypothesis.score < object_detection_confidence) {
              continue;
            }
            const auto & b = det.bbox;
            const double det_left = b.center.position.x - b.size_x / 2.0;
            const double det_top = b.center.position.y - b.size_y / 2.0;
            const double det_right = det_left + b.size_x;
            const double det_bottom = det_top + b.size_y;
            if (uv->x >= det_left && uv->x <= det_right && uv->y >= det_top && uv->y <= det_bottom) {
              const bool is_seed_neighbor_match = (seeded_det_idx >= 0 && static_cast<int>(d) != seeded_det_idx);
              const double combined_score =
                min_iou - (is_seed_neighbor_match ? kSeedNeighborMatchPenalty : 0.0);
              pairs.push_back({c, static_cast<int>(d), min_iou, combined_score});
              break;
            }
          }
        }
      }
    }
  }

  std::unordered_set<size_t> used_candidates;
  std::unordered_set<int> used_detections;
  std::vector<Pair> assignments;
  if (!pairs.empty()) {
    bool all_candidates_roi_seeded = !candidates.empty();
    for (const auto & cand : candidates) {
      if (!cand.roi_seed_det_idx.has_value()) {
        all_candidates_roi_seeded = false;
        break;
      }
    }

    if (all_candidates_roi_seeded) {
      std::sort(
        pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) {
          if (a.combined_score != b.combined_score) return a.combined_score > b.combined_score;
          if (a.iou != b.iou) return a.iou > b.iou;
          if (a.cand_idx != b.cand_idx) return a.cand_idx < b.cand_idx;
          return a.det_idx < b.det_idx;
        });

      for (const auto & p : pairs) {
        if (used_candidates.count(p.cand_idx) || used_detections.count(p.det_idx)) {
          continue;
        }
        used_candidates.insert(p.cand_idx);
        used_detections.insert(p.det_idx);
        assignments.push_back(p);
      }
    } else {
      // Non-ROI/global candidate mode retains global one-to-one matching.
      const size_t nc = candidates.size();
      const size_t nd = detections.detections.size();
      const size_t dim = std::max(nc, nd);
      constexpr double kInvalidPairCost = 1e9;
      constexpr double kInvalidPairCostThreshold = 1e8;

      std::vector<double> cost(dim * dim, kInvalidPairCost);
      std::unordered_map<uint64_t, double> pair_iou;
      pair_iou.reserve(pairs.size());

      auto pairKey = [dim](size_t c, size_t d) -> uint64_t {
          return static_cast<uint64_t>(c) * static_cast<uint64_t>(dim) + static_cast<uint64_t>(d);
        };

      for (const auto & p : pairs) {
        const size_t det_idx = static_cast<size_t>(p.det_idx);
        cost[p.cand_idx * dim + det_idx] = -p.combined_score;
        pair_iou[pairKey(p.cand_idx, det_idx)] = p.iou;
      }

      const std::vector<int> assignment = hungarian::solve(cost, dim);
      for (size_t c = 0; c < nc; ++c) {
        const int d = assignment[c];
        if (d < 0) continue;
        const size_t det_idx = static_cast<size_t>(d);
        if (det_idx >= nd) continue;

        const double val = cost[c * dim + det_idx];
        if (val >= kInvalidPairCostThreshold) continue;

        const auto iou_it = pair_iou.find(pairKey(c, det_idx));
        if (iou_it == pair_iou.end()) continue;

        used_candidates.insert(c);
        used_detections.insert(d);
        assignments.push_back({c, d, iou_it->second, -val});
      }
    }
  }

  const bool allow_second_pass = params.enable_second_pass_fallback &&
    !(params.association_strict_matching && params.association_suppress_second_pass_under_strict);
  // Second pass: for unassigned detections, try relaxed IoU or centroid-in-box
  if (allow_second_pass && params.second_pass_min_iou > 0.0) {
    std::vector<int> unassigned_dets;
    for (size_t d = 0; d < detections.detections.size(); ++d) {
      if (used_detections.count(static_cast<int>(d))) continue;
      if (unassigned_dets.size() >= static_cast<size_t>(params.max_unassigned_detections_second_pass)) break;
      unassigned_dets.push_back(static_cast<int>(d));
    }
    struct CandidateProjInfo
    {
      bool centroid_valid{false};
      cv::Point2d centroid_uv{0.0, 0.0};
      std::vector<cv::Point2d> projected_pts;
      cv::Rect2d projected_rect;
      size_t projected_point_count{0};
    };

    const bool can_compute_point_support = static_cast<bool>(cloud && !cloud->empty());
    std::vector<CandidateProjInfo> proj_infos(candidates.size());
    std::vector<std::optional<cv::Rect2d>> cand_aabb_rects(candidates.size());
    std::vector<std::optional<cv::Rect2d>> second_pass_iou_rects(candidates.size());

    auto pointInRect = [](const cv::Point2d & uv, const cv::Rect2d & r) -> bool
    {
      if (r.width <= 0.0 || r.height <= 0.0) return false;
      const double x1 = r.x + r.width;
      const double y1 = r.y + r.height;
      return uv.x >= r.x && uv.x <= x1 && uv.y >= r.y && uv.y <= y1;
    };

    auto expandRectToImage = [&](const cv::Rect2d & r, double expand_fraction) -> cv::Rect2d
    {
      if (r.width <= 0.0 || r.height <= 0.0) return cv::Rect2d();
      if (expand_fraction <= 0.0) return r;

      const double cx = r.x + r.width / 2.0;
      const double cy = r.y + r.height / 2.0;
      const double new_w = r.width * (1.0 + expand_fraction);
      const double new_h = r.height * (1.0 + expand_fraction);
      const double x0 = std::max(0.0, cx - new_w / 2.0);
      const double y0 = std::max(0.0, cy - new_h / 2.0);
      const double x1 = std::min(iw, cx + new_w / 2.0);
      const double y1 = std::min(ih, cy + new_h / 2.0);
      if (x1 <= x0 || y1 <= y0) return cv::Rect2d();
      return cv::Rect2d(x0, y0, x1 - x0, y1 - y0);
    };

    constexpr double kAssocPointRectLowQ = 0.05;
    constexpr double kAssocPointRectHighQ = 0.95;
    // Precompute projection support for all unused candidates.
    for (size_t c = 0; c < candidates.size(); ++c) {
      if (used_candidates.count(c)) continue;
      // AABB projection is used for IoU + size plausibility.
      cand_aabb_rects[c] = projectAABBRect(candidates[c].stats, lidar_to_image, iw, ih);

      // Centroid projection is used for hard gating (inside expanded bbox).
      pcl::PointXYZ centroid_pt;
      centroid_pt.x = candidates[c].stats.centroid.x();
      centroid_pt.y = candidates[c].stats.centroid.y();
      centroid_pt.z = candidates[c].stats.centroid.z();
      auto centroid_uv = projectLidarToCamera(lidar_to_image, centroid_pt);
      if (centroid_uv && centroid_uv->x >= 0 && centroid_uv->x < iw && centroid_uv->y >= 0 && centroid_uv->y < ih) {
        proj_infos[c].centroid_valid = true;
        proj_infos[c].centroid_uv = *centroid_uv;
      }

      auto & info = proj_infos[c];
      if (can_compute_point_support) {
        // Project all points in this cluster (using stored indices) to support inside-bbox scoring.
        info.projected_pts.clear();
        info.projected_pts.reserve(static_cast<size_t>(std::max(0, candidates[c].stats.num_points)));

        double u_min = std::numeric_limits<double>::infinity();
        double v_min = std::numeric_limits<double>::infinity();
        double u_max = -std::numeric_limits<double>::infinity();
        double v_max = -std::numeric_limits<double>::infinity();

        for (int idx : candidates[c].indices.indices) {
          if (!cloud || idx < 0 || static_cast<size_t>(idx) >= cloud->points.size()) continue;
          const auto & pt = cloud->points[static_cast<size_t>(idx)];
          auto uv = projectLidarToCamera(lidar_to_image, pt);
          if (!uv) continue;
          if (uv->x < 0 || uv->x >= iw || uv->y < 0 || uv->y >= ih) continue;
          info.projected_pts.push_back(*uv);
          u_min = std::min(u_min, uv->x);
          v_min = std::min(v_min, uv->y);
          u_max = std::max(u_max, uv->x);
          v_max = std::max(v_max, uv->y);
        }
        info.projected_point_count = info.projected_pts.size();
        if (info.projected_point_count >= 2u) {
          auto rect = percentileRectFromProjectedPoints(info.projected_pts, kAssocPointRectLowQ, kAssocPointRectHighQ);
          if (rect) {
            info.projected_rect = *rect;
          } else if (u_max > u_min && v_max > v_min) {
            info.projected_rect = cv::Rect2d(u_min, v_min, u_max - u_min, v_max - v_min);
          }
        } else if (info.projected_point_count == 1u) {
          info.projected_rect = cv::Rect2d(
            info.projected_pts[0].x - 3.0, info.projected_pts[0].y - 3.0, 6.0, 6.0);
        }
      }

      std::optional<cv::Rect2d> iou_rect;
      if (params.association_use_point_projection_rect_for_iou && can_compute_point_support) {
        if (info.projected_point_count >= 2u && info.projected_rect.width > 0.0 && info.projected_rect.height > 0.0) {
          iou_rect = info.projected_rect;
        } else if (info.projected_point_count == 1u) {
          iou_rect = cv::Rect2d(
            info.projected_pts[0].x - 3.0, info.projected_pts[0].y - 3.0, 6.0, 6.0);
        }
      }
      if (!iou_rect && cloud && !candidates[c].indices.indices.empty()) {
        const Box3D obox = cluster_box::computeClusterBox(cloud, candidates[c].indices);
        iou_rect = projectOrientedBoxImageRect(obox, lidar_to_image, iw, ih);
      }
      if (!iou_rect) {
        iou_rect = cand_aabb_rects[c];
      }
      second_pass_iou_rects[c] = iou_rect;
    }

    for (int det_idx : unassigned_dets) {
      const auto & det = detections.detections[static_cast<size_t>(det_idx)];

      if (!det.results.empty() && det.results[0].hypothesis.score < object_detection_confidence) continue;

      // Second-pass gate by detection confidence (optional).
      const double det_score = det.results.empty() ? 0.0 : det.results[0].hypothesis.score;
      if (params.second_pass_min_det_conf >= 0.0 && det_score < params.second_pass_min_det_conf) continue;

      const std::string det_class_id = det.results.empty() ? std::string() : det.results[0].hypothesis.class_id;
      const bool is_person = det_class_id == "person";
      const bool is_vehicle = det_class_id == "car" || det_class_id == "truck" || det_class_id == "bus";

      const auto & b = det.bbox;
      const cv::Rect2d det_rect(
        b.center.position.x - b.size_x / 2.0, b.center.position.y - b.size_y / 2.0, b.size_x, b.size_y);
      const double det_area = det_rect.area();
      if (params.second_pass_min_det_area_px2 > 0.0 && det_area < params.second_pass_min_det_area_px2) {
        continue;
      }

      const cv::Rect2d expanded_det_rect = expandRectToImage(det_rect, params.second_pass_bbox_expand_fraction);
      if (expanded_det_rect.width <= 0.0 || expanded_det_rect.height <= 0.0) continue;

      Pair best{static_cast<size_t>(-1), -1, 0.0, 0.0};
      double best_score = -1.0;
      double second_best_score = -1.0;

      for (size_t c = 0; c < candidates.size(); ++c) {
        if (used_candidates.count(c)) continue;
        const auto & info = proj_infos[c];

        // Hard gating: centroid must land inside expanded bbox.
        if (!info.centroid_valid) continue;
        if (!pointInRect(info.centroid_uv, expanded_det_rect)) continue;

        // IoU from point hull (matches strict pass when enabled), else oriented-box image hull, else 3D AABB.
        const auto & cand_rect_opt = second_pass_iou_rects[c];
        double iou_val = 0.0;
        if (cand_rect_opt) {
          const cv::Rect2d inter = *cand_rect_opt & det_rect;
          const double inter_area = (inter.width > 0 && inter.height > 0) ? inter.area() : 0.0;
          const double uni = cand_rect_opt->area() + det_rect.area() - inter_area;
          if (uni > 0.0) iou_val = inter_area / uni;
        } else {
          // If we couldn't project any candidate rect, still allow rescue when centroid/support is strong.
          iou_val = params.second_pass_min_iou;
        }
        if (iou_val < params.second_pass_min_iou) continue;

        // Range tiering (for minimum support points and support AND/OR).
        const auto & sc = candidates[c].stats;
        const double range =
          std::sqrt(sc.centroid.x() * sc.centroid.x() + sc.centroid.y() * sc.centroid.y() + sc.centroid.z() * sc.centroid.z());

        // Optional tiny-far rejection (tightens priors rather than lowering IoU aggressively).
        if (params.second_pass_min_det_area_far_px2 > 0.0 && range > params.quality_distance_threshold_far) {
          if (det_area < params.second_pass_min_det_area_far_px2) continue;
        }

        int min_inside_points = params.second_pass_min_inside_points;
        if (range > params.quality_distance_threshold_far) {
          min_inside_points = static_cast<int>(std::floor(min_inside_points * params.second_pass_inside_points_far_scale));
        } else if (range > params.quality_distance_threshold_medium) {
          min_inside_points = static_cast<int>(std::floor(min_inside_points * params.second_pass_inside_points_medium_scale));
        }

        if (is_person) {
          min_inside_points = static_cast<int>(std::floor(min_inside_points * params.second_pass_person_inside_points_scale));
        } else if (is_vehicle) {
          min_inside_points = static_cast<int>(std::floor(min_inside_points * params.second_pass_vehicle_inside_points_scale));
        }
        min_inside_points = std::max(1, min_inside_points);

        double min_inside_fraction = params.second_pass_min_inside_fraction;
        if (is_person) {
          min_inside_fraction *= params.second_pass_person_inside_fraction_scale;
        } else if (is_vehicle) {
          min_inside_fraction *= params.second_pass_vehicle_inside_fraction_scale;
        }
        min_inside_fraction = std::min(1.0, std::max(0.0, min_inside_fraction));

        size_t inside_count = 0;
        double inside_fraction = 0.0;
        if (can_compute_point_support && info.projected_point_count > 0) {
          for (const auto & uv : info.projected_pts) {
            if (pointInRect(uv, expanded_det_rect)) ++inside_count;
          }
          inside_fraction = static_cast<double>(inside_count) / static_cast<double>(info.projected_point_count);
        } else {
          // Fallback: without point support we only have centroid evidence.
          inside_count = 0;
          inside_fraction = 0.0;
        }

        const bool is_far_range = range > params.quality_distance_threshold_far;
        const bool support_ok = !can_compute_point_support ? true :
          (is_far_range ?
            (inside_count >= static_cast<size_t>(min_inside_points) || inside_fraction >= min_inside_fraction) :
            (inside_count >= static_cast<size_t>(min_inside_points) && inside_fraction >= min_inside_fraction));
        if (!support_ok) continue;

        // Size plausibility (2D footprint match between projected candidate and detection bbox).
        double cand_w = 0.0;
        double cand_h = 0.0;
        if (cand_rect_opt && cand_rect_opt->width > 0.0 && cand_rect_opt->height > 0.0) {
          cand_w = cand_rect_opt->width;
          cand_h = cand_rect_opt->height;
        } else if (info.projected_rect.width > 0.0 && info.projected_rect.height > 0.0) {
          cand_w = info.projected_rect.width;
          cand_h = info.projected_rect.height;
        }

        double size_score = 0.0;
        if (b.size_x > 1e-6 && b.size_y > 1e-6 && cand_w > 1e-6 && cand_h > 1e-6) {
          const double rw = cand_w / b.size_x;
          const double rh = cand_h / b.size_y;
          const double w_score = std::min(rw, 1.0 / rw);
          const double h_score = std::min(rh, 1.0 / rh);
          size_score = std::max(0.0, std::min(1.0, w_score * h_score));
        }

        double min_size_score = params.second_pass_min_size_score;
        if (is_person) min_size_score = params.second_pass_person_min_size_score;
        if (is_vehicle) min_size_score = params.second_pass_vehicle_min_size_score;
        if (size_score < min_size_score) continue;

        // Center proximity score: how close projected centroid is to detection center.
        const double det_cx = b.center.position.x;
        const double det_cy = b.center.position.y;
        const double dist_px = std::hypot(info.centroid_uv.x - det_cx, info.centroid_uv.y - det_cy);
        const double det_diag_px = std::hypot(std::max(1.0, b.size_x), std::max(1.0, b.size_y));
        const double dist_norm = det_diag_px > 1e-6 ? dist_px / det_diag_px : dist_px;
        const double center_score = std::max(0.0, std::min(1.0, 1.0 - dist_norm));

        const double combined_score =
          0.35 * iou_val + 0.30 * inside_fraction + 0.20 * center_score + 0.15 * size_score;

        if (combined_score <= -1e9) continue;
        if (combined_score > best_score) {
          second_best_score = best_score;
          best_score = combined_score;
          best = {c, det_idx, iou_val, combined_score};
        } else if (combined_score > second_best_score) {
          second_best_score = combined_score;
        }
      }

      if (best.cand_idx != static_cast<size_t>(-1)) {
        const double second_for_margin = (second_best_score >= 0.0) ? second_best_score : 0.0;
        if (best_score >= params.second_pass_min_combined_score &&
          (best_score - second_for_margin) >= params.second_pass_best_second_margin)
        {
          used_candidates.insert(best.cand_idx);
          used_detections.insert(best.det_idx);
          assignments.push_back(best);
        }
      }
    }
  }

  std::sort(
    assignments.begin(), assignments.end(), [](const Pair & a, const Pair & b) { return a.cand_idx < b.cand_idx; });

  std::vector<double> expected_depths(assignments.size(), -1.0);
  for (size_t i = 0; i < assignments.size(); ++i) {
    const auto & a = assignments[i];
    if (candidates[a.cand_idx].match.has_value()) {
      expected_depths[i] = candidates[a.cand_idx].match->expected_depth;
    }
  }

  std::vector<ClusterCandidate> kept;
  kept.reserve(assignments.size());
  for (size_t i = 0; i < assignments.size(); ++i) {
    const auto & a = assignments[i];
    ClusterCandidate cand = std::move(candidates[a.cand_idx]);
    cand.match = ClusterDetectionMatch{a.cand_idx, a.det_idx, a.iou, a.combined_score, expected_depths[i]};
    kept.push_back(std::move(cand));
  }
  candidates = std::move(kept);
}

visualization_msgs::msg::MarkerArray computeBoundingBox(
  const std::vector<Box3D> & boxes,
  const std::vector<pcl::PointIndices> & cluster_indices,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  if (boxes.size() != cluster_indices.size()) return marker_array;

  int id = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.empty()) continue;

    const auto & box = boxes[i];

    visualization_msgs::msg::Marker bbox_marker;
    bbox_marker.header = header;
    bbox_marker.ns = "bounding_boxes";
    bbox_marker.id = id++;
    bbox_marker.type = visualization_msgs::msg::Marker::CUBE;
    bbox_marker.action = visualization_msgs::msg::Marker::ADD;

    bbox_marker.pose.position.x = box.center.x();
    bbox_marker.pose.position.y = box.center.y();
    bbox_marker.pose.position.z = box.center.z();

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, box.yaw);
    bbox_marker.pose.orientation = tf2::toMsg(quat);

    bbox_marker.scale.x = std::max(0.0f, box.size.x());
    bbox_marker.scale.y = std::max(0.0f, box.size.y());
    bbox_marker.scale.z = std::max(0.0f, box.size.z());

    const auto & p_viz = getParams();
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = p_viz.marker_alpha;

    bbox_marker.lifetime = rclcpp::Duration::from_seconds(p_viz.marker_lifetime_s);

    marker_array.markers.push_back(bbox_marker);
  }

  return marker_array;
}

namespace
{
constexpr double kDefaultDetectionScore = 1.0;
}

vision_msgs::msg::Detection3DArray compute3DDetection(
  const std::vector<Box3D> & boxes,
  const std::vector<ClusterCandidate> & candidates,
  const std_msgs::msg::Header & header,
  const vision_msgs::msg::Detection2DArray & detections)
{
  vision_msgs::msg::Detection3DArray det_arr;
  det_arr.header = header;

  if (boxes.size() != candidates.size()) return det_arr;

  const auto & params = getParams();

  for (size_t i = 0; i < candidates.size(); ++i) {
    if (candidates[i].indices.indices.empty()) continue;

    const auto & box = boxes[i];

    vision_msgs::msg::Detection3D det;
    det.header = header;

    vision_msgs::msg::ObjectHypothesisWithPose hypo;
    if (candidates[i].match.has_value()) {
      const auto & m = candidates[i].match.value();
      if (m.det_idx >= 0 && static_cast<size_t>(m.det_idx) < detections.detections.size()) {
        const auto & d = detections.detections[static_cast<size_t>(m.det_idx)];
        if (!d.results.empty()) {
          hypo.hypothesis.class_id = d.results[0].hypothesis.class_id;
          if (m.association_score >= 0.0) {
            hypo.hypothesis.score = m.association_score;
          } else {
            const double det_score = static_cast<double>(d.results[0].hypothesis.score);
            hypo.hypothesis.score = params.detection_score_weight * det_score + params.iou_score_weight * m.iou;
          }
          hypo.hypothesis.score = std::max(0.0, std::min(1.0, hypo.hypothesis.score));
        } else {
          hypo.hypothesis.class_id = "cluster";
          hypo.hypothesis.score = kDefaultDetectionScore;
        }
      } else {
        hypo.hypothesis.class_id = "cluster";
        hypo.hypothesis.score = kDefaultDetectionScore;
      }
    } else {
      hypo.hypothesis.class_id = "cluster";
      hypo.hypothesis.score = kDefaultDetectionScore;
    }
    det.results.push_back(hypo);

    geometry_msgs::msg::Pose & pose = det.bbox.center;
    pose.position.x = box.center.x();
    pose.position.y = box.center.y();
    pose.position.z = box.center.z();

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, box.yaw);
    pose.orientation = tf2::toMsg(quat);

    det.bbox.size.x = box.size.x();
    det.bbox.size.y = box.size.y();
    det.bbox.size.z = box.size.z();

    det_arr.detections.push_back(det);
  }

  return det_arr;
}

}  // namespace projection_utils
