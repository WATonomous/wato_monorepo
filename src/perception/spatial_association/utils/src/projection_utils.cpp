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
#include <random>
#include <string>
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

void multiBandClusterExtraction(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double base_cluster_tolerance,
  int maxClusterSize,
  std::vector<pcl::PointIndices> & cluster_indices,
  const std::vector<ClusteringBand> & bands)
{
  cluster_indices.clear();
  if (!cloud || cloud->empty() || bands.empty()) return;

  // Split points into bands by distance from origin.
  const size_t num_bands = bands.size();
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> band_clouds(num_bands);
  std::vector<std::vector<int>> band_index_maps(num_bands);

  for (size_t b = 0; b < num_bands; ++b) {
    band_clouds[b].reset(new pcl::PointCloud<pcl::PointXYZ>);
    band_clouds[b]->reserve(cloud->size() / num_bands);
    band_index_maps[b].reserve(cloud->size() / num_bands);
  }

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto & pt = cloud->points[i];
    double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    // Find which band this point belongs to.
    double prev_max = 0.0;
    for (size_t b = 0; b < num_bands; ++b) {
      if (dist >= prev_max && (dist < bands[b].max_distance || b == num_bands - 1)) {
        band_clouds[b]->points.push_back(pt);
        band_index_maps[b].push_back(static_cast<int>(i));
        break;
      }
      prev_max = bands[b].max_distance;
    }
  }

  // Cluster each band independently and remap indices.
  for (size_t b = 0; b < num_bands; ++b) {
    auto & bc = band_clouds[b];
    bc->width = bc->points.size();
    bc->height = 1;
    bc->is_dense = false;

    if (bc->size() < 2u) continue;

    double tolerance = base_cluster_tolerance * bands[b].tolerance_mult;
    int min_size = bands[b].min_cluster_size;

    std::vector<pcl::PointIndices> band_clusters;
    euclideanClusterExtraction(bc, tolerance, min_size, maxClusterSize, band_clusters);

    for (auto & cluster : band_clusters) {
      pcl::PointIndices mapped;
      mapped.indices.reserve(cluster.indices.size());
      for (int idx : cluster.indices) {
        mapped.indices.push_back(band_index_maps[b][idx]);
      }
      cluster_indices.push_back(std::move(mapped));
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

enum class DetectionClassKind
{
  kUnknown,
  kPerson,
  kVehicle
};

DetectionClassKind classifyDetectionClassId(const std::string & class_id)
{
  if (class_id == "person") return DetectionClassKind::kPerson;
  if (class_id == "car" || class_id == "truck" || class_id == "bus") return DetectionClassKind::kVehicle;
  return DetectionClassKind::kUnknown;
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
}  // namespace

void filterCandidatesByClassAwareConstraints(
  std::vector<ClusterCandidate> & candidates, const vision_msgs::msg::Detection2DArray & detections)
{
  constexpr float kMinDimensionForAspect = 0.05f;
  constexpr float kVolumeThresholdDensity = 0.01f;

  const auto & q = getParams();
  std::vector<DetectionClassKind> detection_class_kinds(detections.detections.size(), DetectionClassKind::kUnknown);
  for (size_t di = 0; di < detections.detections.size(); ++di) {
    const auto & det = detections.detections[di];
    if (det.results.empty()) continue;
    detection_class_kinds[di] = classifyDetectionClassId(det.results[0].hypothesis.class_id);
  }

  size_t write_idx = 0;
  for (size_t i = 0; i < candidates.size(); ++i) {
    auto & cand = candidates[i];
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
      if (density < q.quality_min_density) continue;
    }

    float max_dim = std::max({width_x, width_y, height});
    if (max_dim > q.quality_max_dimension) continue;
    float min_dim = std::min({width_x, width_y, height});
    if (min_dim > kMinDimensionForAspect && max_dim / min_dim > q.quality_max_aspect_ratio) continue;

    if (distance > q.quality_max_distance) continue;

    if (cand.match.has_value()) {
      const int di = cand.match->det_idx;
      if (di >= 0 && static_cast<size_t>(di) < detection_class_kinds.size()) {
        const DetectionClassKind class_kind = detection_class_kinds[static_cast<size_t>(di)];
        const bool is_person = class_kind == DetectionClassKind::kPerson;
        const bool is_vehicle = class_kind == DetectionClassKind::kVehicle;
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
          if (q.quality_person_max_volume_m3 > 0.0f && volume > q.quality_person_max_volume_m3) continue;
        }
      }
    }

    if (write_idx != i) {
      candidates[write_idx] = std::move(cand);
    }
    ++write_idx;
  }
  candidates.resize(write_idx);
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
  int image_height,
  const std::vector<std::optional<double>> & detection_depths)
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

  // First-pass assignment sorts by combined_score (not IoU alone) so stronger point/center
  // support can beat a slightly higher-IoU split overlap. Base weights sum to 1; when depth is
  // available for a specific pair, the base contribution is scaled down to make room for the depth
  // term. When depth is absent the base score is used directly so all pairs are on the same scale.
  const bool has_depths = !detection_depths.empty();
  const double depth_w = params.depth_score_weight;
  const double kAssocWIoU = params.association_weight_iou;
  const double kAssocWInsideFrac = params.association_weight_inside_fraction;
  const double kAssocWAr = params.association_weight_ar;
  const double kAssocWCentroid = params.association_weight_centroid;
  const double kAssocWPoints = params.association_weight_points;

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
        const double sp_margin = params.single_point_bbox_margin_px;
        g.point_rect = cv::Rect2d(g.uvs[0].x - sp_margin, g.uvs[0].y - sp_margin, 2.0 * sp_margin, 2.0 * sp_margin);
      }
    }

    for (size_t c = 0; c < candidates.size(); ++c) {
      const CandGeom & g = geoms[c];
      const auto & st = candidates[c].stats;

      // Early rejection: skip candidates with no valid projection at all.
      if (!g.centroid_ok && g.uvs.empty()) continue;

      // Compute cluster distance for distance-adaptive thresholds.
      const double cluster_dist_2d = std::sqrt(
        st.centroid.x() * st.centroid.x() + st.centroid.y() * st.centroid.y());
      double effective_min_iou = min_iou;
      double effective_min_inside_frac = params.association_min_inside_point_fraction;
      if (cluster_dist_2d > params.quality_distance_threshold_far) {
        effective_min_iou *= params.far_iou_threshold_scale;
        effective_min_inside_frac *= params.far_inside_fraction_scale;
      } else if (cluster_dist_2d > params.quality_distance_threshold_medium) {
        effective_min_iou *= params.medium_iou_threshold_scale;
        effective_min_inside_frac *= params.medium_inside_fraction_scale;
      }

      for (int d = 0; d < static_cast<int>(detections.detections.size()); ++d) {
        const auto & det = detections.detections[static_cast<size_t>(d)];
        if (!det.results.empty() && det.results[0].hypothesis.score < object_detection_confidence) {
          continue;
        }
        const auto & b = det.bbox;

        // Early rejection: skip if centroid is far from detection center.
        if (g.centroid_ok) {
          const double max_dim = params.association_centroid_distance_multiplier * std::max(b.size_x, b.size_y);
          const double dx = g.centroid_uv.x - b.center.position.x;
          const double dy = g.centroid_uv.y - b.center.position.y;
          if (dx * dx + dy * dy > max_dim * max_dim) continue;
        }

        // Class-aware threshold scaling (on top of distance-adaptive).
        double det_min_iou = effective_min_iou;
        double det_min_inside_frac = effective_min_inside_frac;
        double det_min_ar = params.association_min_ar_consistency_score;
        if (!det.results.empty()) {
          const auto & cls = det.results[0].hypothesis.class_id;
          if (cls == "person") {
            det_min_iou *= params.person_iou_threshold_scale;
            det_min_inside_frac *= params.person_inside_fraction_scale;
            det_min_ar *= params.person_ar_consistency_scale;
          } else if (cls == "car" || cls == "truck" || cls == "bus") {
            det_min_iou *= params.vehicle_iou_threshold_scale;
            det_min_inside_frac *= params.vehicle_inside_fraction_scale;
          }
        }

        const cv::Rect2d det_rect(
          b.center.position.x - b.size_x / 2.0, b.center.position.y - b.size_y / 2.0, b.size_x, b.size_y);

        // Far-range detection area gate: reject tiny far detections (unreliable).
        const bool is_far = cluster_dist_2d > params.quality_distance_threshold_far;
        if (is_far && params.min_det_area_far_px2 > 0.0) {
          if (b.size_x * b.size_y < params.min_det_area_far_px2) continue;
        }

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

        const cv::Rect2d inter_rect = *cand_rect & det_rect;
        const double inter_area = (inter_rect.width > 0.0 && inter_rect.height > 0.0) ? inter_rect.area() : 0.0;
        const double uni = cand_rect->area() + det_rect.area() - inter_area;
        if (uni <= 0.0) continue;
        const double iou = inter_area / uni;
        if (iou < det_min_iou) continue;

        // Inside-point support with absolute count floor + OR-at-far-range.
        if (g.uvs.empty()) continue;
        size_t inside = 0;
        for (const auto & uv : g.uvs) {
          if (pointInRectClosed(uv, det_rect)) ++inside;
        }
        const double inside_frac = static_cast<double>(inside) / static_cast<double>(g.uvs.size());
        const bool count_ok = static_cast<int>(inside) >= params.association_min_inside_points;
        const bool frac_ok = inside_frac >= det_min_inside_frac;
        if (is_far) {
          if (!count_ok && !frac_ok) continue;   // OR at far range (lenient)
        } else {
          if (!count_ok || !frac_ok) continue;    // AND at close/medium range (strict)
        }

        if (st.num_points < params.quality_min_points) continue;
        if (st.num_points < qualityTieredMinPoints(st, params)) continue;

        const double cand_ar = cand_rect->width / std::max(1e-6, cand_rect->height);
        const double det_ar = b.size_x / std::max(1e-6, b.size_y);
        const double rr = cand_ar / std::max(1e-9, det_ar);
        const double ar_score = std::min(rr, 1.0 / rr);
        if (ar_score < det_min_ar) continue;

        // Size consistency: independent width/height ratio plausibility.
        double size_score = 1.0;
        if (cand_rect->width > 1e-6 && cand_rect->height > 1e-6 && b.size_x > 1e-6 && b.size_y > 1e-6) {
          const double rw = cand_rect->width / b.size_x;
          const double rh = cand_rect->height / b.size_y;
          size_score = std::min(rw, 1.0 / rw) * std::min(rh, 1.0 / rh);
          if (size_score < params.min_size_consistency_score) continue;
        }
        const double det_cx = b.center.position.x;
        const double det_cy = b.center.position.y;
        const double dist_c = std::hypot(g.centroid_uv.x - det_cx, g.centroid_uv.y - det_cy);
        const double det_scale = std::max(1e-3, params.centroid_score_detection_scale * std::hypot(b.size_x, b.size_y));
        const double centroid_score = std::exp(-dist_c / det_scale);
        const double point_score =
          std::min(1.0, std::log(1.0 + static_cast<double>(st.num_points)) / std::log(1.0 + params.point_score_saturation_count));
        const double base_score = kAssocWIoU * iou + kAssocWInsideFrac * inside_frac + kAssocWAr * ar_score +
                                   kAssocWCentroid * centroid_score + kAssocWPoints * point_score;
        double combined_score;
        if (
          has_depths && static_cast<size_t>(d) < detection_depths.size() &&
          detection_depths[static_cast<size_t>(d)].has_value())
        {
          const double det_depth = detection_depths[static_cast<size_t>(d)].value();
          const double cluster_dist = st.centroid.head<3>().norm();
          const double ds = depth_w * std::exp(-std::abs(cluster_dist - det_depth) / params.depth_score_scale);
          combined_score = base_score * (1.0 - depth_w) + ds;
        } else {
          combined_score = base_score;
        }

        pairs.push_back({c, d, iou, combined_score});
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
              pairs.push_back({c, static_cast<int>(d), iou, iou});
            }
          }
        }
      }
    }
  }

  std::vector<char> used_candidates(candidates.size(), 0);
  std::vector<char> used_detections(detections.detections.size(), 0);
  std::vector<Pair> assignments;

  if (params.use_hungarian_assignment && !pairs.empty()) {
    // Build a score lookup from the valid pairs.
    std::unordered_map<size_t, std::unordered_map<int, Pair>> pair_lookup;
    for (const auto & p : pairs) {
      auto it = pair_lookup[p.cand_idx].find(p.det_idx);
      if (it == pair_lookup[p.cand_idx].end() || p.combined_score > it->second.combined_score) {
        pair_lookup[p.cand_idx][p.det_idx] = p;
      }
    }

    // Collect unique candidate and detection indices.
    std::vector<size_t> cand_ids;
    std::unordered_set<int> det_id_set;
    for (const auto & kv : pair_lookup) {
      cand_ids.push_back(kv.first);
      for (const auto & inner : kv.second) {
        det_id_set.insert(inner.first);
      }
    }
    std::sort(cand_ids.begin(), cand_ids.end());
    std::vector<int> det_ids(det_id_set.begin(), det_id_set.end());
    std::sort(det_ids.begin(), det_ids.end());

    const size_t nr = cand_ids.size();
    const size_t nc = det_ids.size();
    const double kForbidden = 2.0; // scores are in [0,1], so cost 2.0 = forbidden

    // Build cost matrix: cost = 1.0 - combined_score (forbidden = kForbidden).
    std::vector<double> cost_matrix(nr * nc, kForbidden);
    for (size_t ri = 0; ri < nr; ++ri) {
      auto cit = pair_lookup.find(cand_ids[ri]);
      if (cit == pair_lookup.end()) continue;
      for (size_t ci = 0; ci < nc; ++ci) {
        auto dit = cit->second.find(det_ids[ci]);
        if (dit != cit->second.end()) {
          cost_matrix[ri * nc + ci] = 1.0 - dit->second.combined_score;
        }
      }
    }

    auto result = hungarian::solve(cost_matrix, nr, nc, kForbidden);
    for (const auto & [ri, ci] : result) {
      size_t real_cand = cand_ids[ri];
      int real_det = det_ids[ci];
      auto cit = pair_lookup.find(real_cand);
      if (cit != pair_lookup.end()) {
        auto dit = cit->second.find(real_det);
        if (dit != cit->second.end()) {
          used_candidates[real_cand] = 1;
          used_detections[static_cast<size_t>(real_det)] = 1;
          assignments.push_back(dit->second);
        }
      }
    }
  } else {
    // Greedy fallback.
    std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) {
      if (a.combined_score != b.combined_score) {
        return a.combined_score > b.combined_score;
      }
      return a.iou > b.iou;
    });

    for (const auto & p : pairs) {
      if (used_candidates[p.cand_idx]) continue;
      if (p.det_idx < 0 || static_cast<size_t>(p.det_idx) >= used_detections.size()) continue;
      if (used_detections[static_cast<size_t>(p.det_idx)]) continue;
      used_candidates[p.cand_idx] = 1;
      used_detections[static_cast<size_t>(p.det_idx)] = 1;
      assignments.push_back(p);
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

    if (candidates[i].match.has_value()) {
      const auto & m = candidates[i].match.value();
      if (m.det_idx >= 0 && static_cast<size_t>(m.det_idx) < detections.detections.size()) {
        const auto & d = detections.detections[static_cast<size_t>(m.det_idx)];
        if (!d.results.empty()) {
          // Copy all hypotheses (class + attributes like state:red, behavior:braking)
          det.results = d.results;
          // Recalculate score on the primary hypothesis (index 0)
          const double det_score = static_cast<double>(d.results[0].hypothesis.score);
          det.results[0].hypothesis.score =
            std::max(0.0, std::min(1.0, params.detection_score_weight * det_score + params.iou_score_weight * m.iou));
        } else {
          vision_msgs::msg::ObjectHypothesisWithPose hypo;
          hypo.hypothesis.class_id = "cluster";
          hypo.hypothesis.score = kDefaultDetectionScore;
          det.results.push_back(hypo);
        }
      } else {
        vision_msgs::msg::ObjectHypothesisWithPose hypo;
        hypo.hypothesis.class_id = "cluster";
        hypo.hypothesis.score = kDefaultDetectionScore;
        det.results.push_back(hypo);
      }
    } else {
      vision_msgs::msg::ObjectHypothesisWithPose hypo;
      hypo.hypothesis.class_id = "cluster";
      hypo.hypothesis.score = kDefaultDetectionScore;
      det.results.push_back(hypo);
    }

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
