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
    candidates.push_back(ClusterCandidate{cluster_indices[i], std::move(stats[i]), std::nullopt});
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

void adaptiveEuclideanClusterExtraction(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double base_cluster_tolerance,
  int minClusterSize,
  int maxClusterSize,
  std::vector<pcl::PointIndices> & cluster_indices,
  double close_threshold,
  double close_tolerance_mult)
{
  if (!cloud || cloud->empty()) {
    cluster_indices.clear();
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr close_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> close_indices_map;
  std::vector<int> far_indices_map;

  close_cloud->reserve(cloud->size());
  far_cloud->reserve(cloud->size());
  close_indices_map.reserve(cloud->size());
  far_indices_map.reserve(cloud->size());

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto & pt = cloud->points[i];
    double distance = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

    if (distance < close_threshold) {
      close_cloud->points.push_back(pt);
      close_indices_map.push_back(static_cast<int>(i));
    } else {
      far_cloud->points.push_back(pt);
      far_indices_map.push_back(static_cast<int>(i));
    }
  }

  close_cloud->width = close_cloud->points.size();
  close_cloud->height = 1;
  close_cloud->is_dense = false;
  far_cloud->width = far_cloud->points.size();
  far_cloud->height = 1;
  far_cloud->is_dense = false;

  cluster_indices.clear();

  if (close_cloud->size() >= 2u) {
    double close_tolerance = base_cluster_tolerance * close_tolerance_mult;
    std::vector<pcl::PointIndices> close_clusters;
    euclideanClusterExtraction(close_cloud, close_tolerance, minClusterSize, maxClusterSize, close_clusters);

    for (auto & cluster : close_clusters) {
      pcl::PointIndices mapped_cluster;
      mapped_cluster.indices.reserve(cluster.indices.size());
      for (int idx : cluster.indices) {
        mapped_cluster.indices.push_back(close_indices_map[idx]);
      }
      cluster_indices.push_back(mapped_cluster);
    }
  }

  if (far_cloud->size() >= 2u) {
    std::vector<pcl::PointIndices> far_clusters;
    euclideanClusterExtraction(far_cloud, base_cluster_tolerance, minClusterSize, maxClusterSize, far_clusters);

    for (auto & cluster : far_clusters) {
      pcl::PointIndices mapped_cluster;
      mapped_cluster.indices.reserve(cluster.indices.size());
      for (int idx : cluster.indices) {
        mapped_cluster.indices.push_back(far_indices_map[idx]);
      }
      cluster_indices.push_back(mapped_cluster);
    }
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

  std::vector<char> claimed(cloud->points.size(), 0);

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
    const cv::Rect2d det_rect(
      b.center.position.x - b.size_x / 2.0, b.center.position.y - b.size_y / 2.0, b.size_x, b.size_y);
    const cv::Rect2d roi_rect = expandRectToImageBounds(det_rect, roi_expand_fraction, iw, ih);
    if (roi_rect.width <= 0.0 || roi_rect.height <= 0.0) {
      continue;
    }

    std::vector<int> roi_globals;
    roi_globals.reserve(cloud->size() / 8u + 8u);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      if (claim_points_unique && claimed[i]) continue;
      const auto & pt = cloud->points[i];
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      auto uv = projectLidarToCamera(lidar_to_image, pt);
      if (!uv) continue;
      if (uv->x < 0.0 || uv->x >= iw || uv->y < 0.0 || uv->y >= ih) continue;
      if (!pointInRectClosed(*uv, roi_rect)) continue;
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

    int best_k = -1;
    int best_inside = -1;
    for (size_t k = 0; k < clusters.size(); ++k) {
      int inside = 0;
      for (int gi : clusters[k].indices) {
        const auto & pt = cloud->points[static_cast<size_t>(gi)];
        auto uv = projectLidarToCamera(lidar_to_image, pt);
        if (!uv) continue;
        if (uv->x < 0.0 || uv->x >= iw || uv->y < 0.0 || uv->y >= ih) continue;
        if (pointInRectClosed(*uv, det_rect)) ++inside;
      }
      if (inside > best_inside) {
        best_inside = inside;
        best_k = static_cast<int>(k);
      }
    }
    if (best_k < 0 || static_cast<int>(clusters[static_cast<size_t>(best_k)].indices.size()) < min_cluster_size) {
      continue;
    }

    ClusterCandidate cand;
    cand.indices = std::move(clusters[static_cast<size_t>(best_k)]);
    cand.stats = cluster_box::computeSingleClusterStats(cloud, cand.indices);
    cand.match = ClusterDetectionMatch{0, det_idx, 1.0};

    if (claim_points_unique) {
      for (int gi : cand.indices.indices) {
        if (gi >= 0 && static_cast<size_t>(gi) < claimed.size()) {
          claimed[static_cast<size_t>(gi)] = 1;
        }
      }
    }
    result.push_back(std::move(cand));
  }
  return result;
}

void filterCandidatesByClassAwareConstraints(
  std::vector<ClusterCandidate> & candidates, const vision_msgs::msg::Detection2DArray & detections)
{
  constexpr float kMinDimensionForAspect = 0.05f;
  constexpr float kVolumeThresholdDensity = 0.01f;

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

    int min_points_threshold = qualityTieredMinPoints(s, q);
    if (s.num_points < min_points_threshold) continue;

    if (volume > kVolumeThresholdDensity) {
      float density = s.num_points / volume;
      if (density < q.quality_min_density || density > q.quality_max_density) continue;
    }

    float max_dim = std::max({width_x, width_y, height});
    if (max_dim > q.quality_max_dimension) continue;
    float min_dim = std::min({width_x, width_y, height});
    if (min_dim > kMinDimensionForAspect && max_dim / min_dim > q.quality_max_aspect_ratio) continue;

    if (distance > q.quality_max_distance) continue;

    if (cand.match.has_value()) {
      const int di = cand.match->det_idx;
      if (di >= 0 && static_cast<size_t>(di) < detections.detections.size()) {
        const auto & det = detections.detections[static_cast<size_t>(di)];
        const std::string cid = det.results.empty() ? std::string() : det.results[0].hypothesis.class_id;
        const bool is_person = cid == "person";
        const bool is_vehicle = cid == "car" || cid == "truck" || cid == "bus";
        float cap_h = q.quality_max_dimension;
        float cap_xy = q.quality_max_dimension;
        if (is_person) {
          cap_h = std::min(cap_h, q.quality_person_max_height_m);
          cap_xy = std::min(cap_xy, q.quality_person_max_footprint_xy_m);
        } else if (is_vehicle) {
          cap_h = std::min(cap_h, q.quality_vehicle_max_height_m);
        }
        if (height > cap_h) continue;
        if (is_person) {
          const float foot = std::max(width_x, width_y);
          if (foot > cap_xy) continue;
        }
      }
    }

    kept.push_back(std::move(cand));
  }
  candidates = std::move(kept);
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

  struct Pair
  {
    size_t cand_idx;
    int det_idx;
    double iou;
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

  if (params.association_strict_matching) {
    struct CandGeom
    {
      std::vector<cv::Point2d> uvs;
      bool centroid_ok{false};
      cv::Point2d centroid_uv{0.0, 0.0};
      std::optional<cv::Rect2d> point_rect;
      std::optional<cv::Rect2d> aabb_rect;
    };
    std::vector<CandGeom> geoms(candidates.size());
    for (size_t c = 0; c < candidates.size(); ++c) {
      CandGeom & g = geoms[c];
      g.aabb_rect = projectAABBRect(candidates[c].stats, lidar_to_image, iw, ih);
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
        g.point_rect = cv::Rect2d(u0, v0, u1 - u0, v1 - v0);
      } else if (g.uvs.size() == 1u) {
        g.point_rect = cv::Rect2d(g.uvs[0].x - 3.0, g.uvs[0].y - 3.0, 6.0, 6.0);
      }
    }

    for (size_t c = 0; c < candidates.size(); ++c) {
      const CandGeom & g = geoms[c];
      const auto & st = candidates[c].stats;
      for (int d = 0; d < static_cast<int>(detections.detections.size()); ++d) {
        const auto & det = detections.detections[static_cast<size_t>(d)];
        if (!det.results.empty() && det.results[0].hypothesis.score < object_detection_confidence) {
          continue;
        }
        const auto & b = det.bbox;
        const cv::Rect2d det_rect(
          b.center.position.x - b.size_x / 2.0, b.center.position.y - b.size_y / 2.0, b.size_x, b.size_y);
        const cv::Rect2d inner = shrinkInner(det_rect, params.association_centroid_inner_box_fraction);
        if (inner.width <= 0.0 || inner.height <= 0.0) continue;
        if (!g.centroid_ok || !pointInRectClosed(g.centroid_uv, inner)) continue;

        std::optional<cv::Rect2d> cand_rect;
        if (params.association_use_point_projection_rect_for_iou && g.point_rect) {
          cand_rect = g.point_rect;
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
        if (static_cast<double>(inside) <
            params.association_min_inside_point_fraction * static_cast<double>(g.uvs.size()))
        {
          continue;
        }

        if (st.num_points < params.quality_min_points) continue;
        if (st.num_points < qualityTieredMinPoints(st, params)) continue;

        const double cand_ar = cand_rect->width / std::max(1e-6, cand_rect->height);
        const double det_ar = b.size_x / std::max(1e-6, b.size_y);
        const double rr = cand_ar / std::max(1e-9, det_ar);
        const double ar_score = std::min(rr, 1.0 / rr);
        if (ar_score < params.association_min_ar_consistency_score) continue;

        pairs.push_back({c, d, iou});
      }
    }
  } else {
    for (size_t c = 0; c < candidates.size(); ++c) {
      auto cluster_rect = projectAABBRect(candidates[c].stats, lidar_to_image, iw, ih);

      if (cluster_rect) {
        for (size_t d = 0; d < detections.detections.size(); ++d) {
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
              pairs.push_back({c, static_cast<int>(d), iou});
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
              pairs.push_back({c, static_cast<int>(d), min_iou});
              break;
            }
          }
        }
      }
    }
  }

  std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) { return a.iou > b.iou; });

  std::unordered_set<size_t> used_candidates;
  std::unordered_set<int> used_detections;
  std::vector<Pair> assignments;
  for (const auto & p : pairs) {
    if (used_candidates.count(p.cand_idx) || used_detections.count(p.det_idx)) continue;
    used_candidates.insert(p.cand_idx);
    used_detections.insert(p.det_idx);
    assignments.push_back(p);
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

      if (!can_compute_point_support) continue;

      // Project all points in this cluster (using stored indices) to support inside-bbox scoring.
      auto & info = proj_infos[c];
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
      if (info.projected_point_count > 0) {
        info.projected_rect = cv::Rect2d(u_min, v_min, u_max - u_min, v_max - v_min);
      }
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

      Pair best{static_cast<size_t>(-1), -1, 0.0};
      double best_score = -1.0;
      double second_best_score = -1.0;

      for (size_t c = 0; c < candidates.size(); ++c) {
        if (used_candidates.count(c)) continue;
        const auto & info = proj_infos[c];

        // Hard gating: centroid must land inside expanded bbox.
        if (!info.centroid_valid) continue;
        if (!pointInRect(info.centroid_uv, expanded_det_rect)) continue;

        // Compute IoU from projected candidate AABB (relaxed IoU is part of combined score, not the only gate).
        const auto & cand_rect_opt = cand_aabb_rects[c];
        double iou_val = 0.0;
        if (cand_rect_opt) {
          const cv::Rect2d inter = *cand_rect_opt & det_rect;
          const double inter_area = (inter.width > 0 && inter.height > 0) ? inter.area() : 0.0;
          const double uni = cand_rect_opt->area() + det_rect.area() - inter_area;
          if (uni > 0.0) iou_val = inter_area / uni;
        } else {
          // If we couldn't project AABB, still allow rescue when centroid/support is strong.
          iou_val = params.second_pass_min_iou;
        }
        if (iou_val < params.second_pass_min_iou) continue;

        // Range tiering (for minimum support points).
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

        const bool support_ok = can_compute_point_support ?
          (inside_count >= static_cast<size_t>(min_inside_points) || inside_fraction >= min_inside_fraction) :
          true;
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
          best = {c, det_idx, iou_val};
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

  std::vector<ClusterCandidate> kept;
  kept.reserve(assignments.size());
  for (const auto & a : assignments) {
    ClusterCandidate cand = std::move(candidates[a.cand_idx]);
    cand.match = ClusterDetectionMatch{a.cand_idx, a.det_idx, a.iou};
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
          const double det_score = static_cast<double>(d.results[0].hypothesis.score);
          hypo.hypothesis.score = params.detection_score_weight * det_score + params.iou_score_weight * m.iou;
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
