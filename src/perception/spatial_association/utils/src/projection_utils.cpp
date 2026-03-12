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

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "pcl/filters/voxel_grid.h"

// Static params storage; set by node from config, read by all projection/utils logic
ProjectionUtils::ProjectionUtilsParams ProjectionUtils::s_params_{};

void ProjectionUtils::setParams(const ProjectionUtilsParams & params)
{
  s_params_ = params;
}

const ProjectionUtils::ProjectionUtilsParams & ProjectionUtils::getParams()
{
  return s_params_;
}

namespace
{
// Default detection confidence score (0.0-1.0) - used only in compute3DDetection
constexpr double kDefaultDetectionScore = 1.0;

/** Linear interpolation percentile: pct in [0, 100]. Returns {low_val, high_val} for robust extents.
 *  Uses nth_element (O(n)) instead of full sort. Modifies data in place; callers must not use data after. */
void getPercentileBounds(std::vector<double> & data, double low_pct, double high_pct, double & out_low, double & out_high)
{
  if (data.empty()) {
    out_low = out_high = 0.0;
    return;
  }
  const size_t n = data.size();
  if (n == 1u) {
    out_low = out_high = data[0];
    return;
  }
  const double n_d = static_cast<double>(n);
  const double idx_low = (low_pct / 100.0) * (n_d - 1.0);
  const double idx_high = (high_pct / 100.0) * (n_d - 1.0);
  const size_t i0 = static_cast<size_t>(std::floor(idx_low));
  const size_t i1 = std::min(i0 + 1u, n - 1u);
  const size_t j0 = static_cast<size_t>(std::floor(idx_high));
  const size_t j1 = std::min(j0 + 1u, n - 1u);

  // Collect distinct rank indices in descending order; nth_element from right to left so we don't overwrite.
  std::array<size_t, 4> indices = {i0, i1, j0, j1};
  std::sort(indices.begin(), indices.end(), std::greater<size_t>());
  size_t k = 0u;
  for (size_t i = 1u; i < 4u; ++i) {
    if (indices[i] != indices[k]) indices[++k] = indices[i];
  }
  const size_t num_ranks = k + 1u;

  std::unordered_map<size_t, double> rank_val;
  rank_val.reserve(4u);
  auto it = data.begin();
  auto it_end = data.end();
  for (size_t r = 0u; r < num_ranks; ++r) {
    const size_t rank = indices[r];
    std::nth_element(it, it + static_cast<std::ptrdiff_t>(rank), it_end);
    rank_val[rank] = data[rank];
    it_end = it + static_cast<std::ptrdiff_t>(rank) + 1;
  }

  const double t_low = idx_low - std::floor(idx_low);
  const double t_high = idx_high - std::floor(idx_high);
  out_low = rank_val[i0] * (1.0 - t_low) + rank_val[i1] * t_low;
  out_high = rank_val[j0] * (1.0 - t_high) + rank_val[j1] * t_high;
  if (out_low > out_high) std::swap(out_low, out_high);
}

struct SearchResult
{
  Eigen::Vector2f center_xy{0.f, 0.f};
  double yaw{0.0};
  float len{0.f};
  float wid{0.f};
  bool ok{false};
};

inline double normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double angleDiff(double a, double b)
{
  double d = normalizeAngle(a - b);
  return std::abs(d);
}

std::vector<Eigen::Vector2f> samplePointsXY(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const std::vector<int> & indices,
  size_t desired_count)
{
  std::vector<Eigen::Vector2f> pts;
  if (indices.empty()) return pts;

  size_t step = 1;
  if (indices.size() > desired_count) {
    step = indices.size() / desired_count;
    if (step == 0) step = 1;
  }

  pts.reserve((indices.size() + step - 1) / step);
  for (size_t i = 0; i < indices.size(); i += step) {
    const auto & p = cloud.points[indices[i]];
    pts.emplace_back(p.x, p.y);
  }
  return pts;
}

SearchResult computeSearchBasedFit(const pcl::PointCloud<pcl::PointXYZ> & pts, const std::vector<int> & indices)
{
  const auto & p = ProjectionUtils::getParams();
  SearchResult r;
  if (indices.size() < p.min_points_for_fit) {
    r.ok = false;
    return r;
  }

  auto search_points = samplePointsXY(pts, indices, p.default_sample_point_count);
  if (search_points.size() < p.min_points_for_fit) {
    r.ok = false;
    return r;
  }

  double min_energy = std::numeric_limits<double>::max();
  double best_theta = 0.0;

  std::vector<std::pair<double, double>> rotated_cache;
  rotated_cache.resize(search_points.size());

  const double pct_lo = p.xy_extent_percentile_low;
  const double pct_hi = p.xy_extent_percentile_high;

  // Use min/max for energy ranking (O(n) per angle). Percentile bounds are applied only
  // when computing the final box extents below, where robustness to outliers matters.
  auto calculateEdgeEnergy = [&](double theta) -> double {
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x;
    double max_y = max_x;

    for (size_t i = 0; i < search_points.size(); ++i) {
      const auto & pt = search_points[i];
      double x_rot = static_cast<double>(pt.x()) * cos_t + static_cast<double>(pt.y()) * sin_t;
      double y_rot = -static_cast<double>(pt.x()) * sin_t + static_cast<double>(pt.y()) * cos_t;
      rotated_cache[i] = {x_rot, y_rot};
      min_x = std::min(min_x, x_rot);
      max_x = std::max(max_x, x_rot);
      min_y = std::min(min_y, y_rot);
      max_y = std::max(max_y, y_rot);
    }

    double energy = 0.0;
    for (const auto & pr : rotated_cache) {
      double dx = std::min(std::abs(pr.first - min_x), std::abs(pr.first - max_x));
      double dy = std::min(std::abs(pr.second - min_y), std::abs(pr.second - max_y));
      double d = std::min(dx, dy);
      energy += d;
    }

    return energy / static_cast<double>(search_points.size());
  };

  const double step_rad = p.orientation_search_step_degrees * M_PI / 180.0;
  const double coarse_step = 5.0 * step_rad;
  for (double theta = 0.0; theta < M_PI_2; theta += coarse_step) {
    double energy = calculateEdgeEnergy(theta);
    if (energy < min_energy) {
      min_energy = energy;
      best_theta = theta;
    }
  }

  const double fine_range = 2.5 * step_rad;
  const double fine_step = step_rad;

  double start_theta = std::max(0.0, best_theta - fine_range);
  double end_theta = std::min(M_PI_2, best_theta + fine_range);

  for (double theta = start_theta; theta <= end_theta; theta += fine_step) {
    double energy = calculateEdgeEnergy(theta);
    if (energy < min_energy) {
      min_energy = energy;
      best_theta = theta;
    }
  }

  std::vector<Eigen::Vector2f> all_points_2d;
  all_points_2d.reserve(indices.size());
  for (int idx : indices) {
    const auto & pt = pts.points[idx];
    all_points_2d.emplace_back(pt.x, pt.y);
  }

  double cos_t = std::cos(best_theta);
  double sin_t = std::sin(best_theta);

  std::vector<double> x_rot_final, y_rot_final;
  x_rot_final.reserve(all_points_2d.size());
  y_rot_final.reserve(all_points_2d.size());
  for (const auto & pt : all_points_2d) {
    double x_rot = static_cast<double>(pt.x()) * cos_t + static_cast<double>(pt.y()) * sin_t;
    double y_rot = -static_cast<double>(pt.x()) * sin_t + static_cast<double>(pt.y()) * cos_t;
    x_rot_final.push_back(x_rot);
    y_rot_final.push_back(y_rot);
  }
  double min_x, max_x, min_y, max_y;
  getPercentileBounds(x_rot_final, pct_lo, pct_hi, min_x, max_x);
  getPercentileBounds(y_rot_final, pct_lo, pct_hi, min_y, max_y);

  double cx_rot = 0.5 * (min_x + max_x);
  double cy_rot = 0.5 * (min_y + max_y);

  double cx = cx_rot * cos_t - cy_rot * sin_t;
  double cy = cx_rot * sin_t + cy_rot * cos_t;

  r.center_xy = Eigen::Vector2f(static_cast<float>(cx), static_cast<float>(cy));

  double d1 = max_x - min_x;
  double d2 = max_y - min_y;

  if (d1 >= d2) {
    r.len = static_cast<float>(d1);
    r.wid = static_cast<float>(d2);
    r.yaw = normalizeAngle(best_theta);
  } else {
    r.len = static_cast<float>(d2);
    r.wid = static_cast<float>(d1);
    r.yaw = normalizeAngle(best_theta + M_PI_2);
  }

  r.ok = true;
  return r;
}

void recomputeExtentsInYaw(
  const pcl::PointCloud<pcl::PointXYZ> & pts,
  const std::vector<int> & indices,
  double yaw,
  const Eigen::Vector2f & base_center,
  Eigen::Vector2f & out_center,
  Eigen::Vector2f & out_size)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  std::vector<double> xs_rot;
  std::vector<double> ys_rot;
  xs_rot.reserve(indices.size());
  ys_rot.reserve(indices.size());

  double sum_xr = 0.0, sum_yr = 0.0;

  for (int idx : indices) {
    const auto & pt = pts.points[idx];
    double dx = pt.x - base_center.x();
    double dy = pt.y - base_center.y();

    double x_rot = dx * cos_yaw + dy * sin_yaw;
    double y_rot = -dx * sin_yaw + dy * cos_yaw;

    xs_rot.push_back(x_rot);
    ys_rot.push_back(y_rot);
    sum_xr += x_rot;
    sum_yr += y_rot;
  }

  const auto & p_rej = ProjectionUtils::getParams();
  const bool apply_outlier_rejection = (indices.size() < p_rej.outlier_rejection_point_count);
  const double pct_lo = p_rej.xy_extent_percentile_low;
  const double pct_hi = p_rej.xy_extent_percentile_high;

  double min_x_rot, max_x_rot, min_y_rot, max_y_rot;

  if (apply_outlier_rejection) {
    double mean_xr = sum_xr / indices.size();
    double mean_yr = sum_yr / indices.size();

    double var_xr = 0.0, var_yr = 0.0;
    for (size_t i = 0; i < indices.size(); ++i) {
      var_xr += (xs_rot[i] - mean_xr) * (xs_rot[i] - mean_xr);
      var_yr += (ys_rot[i] - mean_yr) * (ys_rot[i] - mean_yr);
    }
    double std_xr = std::sqrt(var_xr / indices.size());
    double std_yr = std::sqrt(var_yr / indices.size());

    const double sigma_mul = p_rej.outlier_sigma_multiplier;

    std::vector<double> x_clipped, y_clipped;
    x_clipped.reserve(indices.size());
    y_clipped.reserve(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      double val_x = xs_rot[i];
      double val_y = ys_rot[i];
      if (val_x > mean_xr + sigma_mul * std_xr) val_x = mean_xr + sigma_mul * std_xr;
      if (val_x < mean_xr - sigma_mul * std_xr) val_x = mean_xr - sigma_mul * std_xr;
      if (val_y > mean_yr + sigma_mul * std_yr) val_y = mean_yr + sigma_mul * std_yr;
      if (val_y < mean_yr - sigma_mul * std_yr) val_y = mean_yr - sigma_mul * std_yr;
      x_clipped.push_back(val_x);
      y_clipped.push_back(val_y);
    }
    getPercentileBounds(x_clipped, pct_lo, pct_hi, min_x_rot, max_x_rot);
    getPercentileBounds(y_clipped, pct_lo, pct_hi, min_y_rot, max_y_rot);
  } else {
    getPercentileBounds(xs_rot, pct_lo, pct_hi, min_x_rot, max_x_rot);
    getPercentileBounds(ys_rot, pct_lo, pct_hi, min_y_rot, max_y_rot);
  }

  double center_x_rot = 0.5 * (min_x_rot + max_x_rot);
  double center_y_rot = 0.5 * (min_y_rot + max_y_rot);

  double center_x = base_center.x() + center_x_rot * cos_yaw - center_y_rot * sin_yaw;
  double center_y = base_center.y() + center_x_rot * sin_yaw + center_y_rot * cos_yaw;

  out_center = Eigen::Vector2f(static_cast<float>(center_x), static_cast<float>(center_y));
  out_size = Eigen::Vector2f(static_cast<float>(max_x_rot - min_x_rot), static_cast<float>(max_y_rot - min_y_rot));
}

ProjectionUtils::Box3D computeClusterBox(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::PointIndices & cluster)
{
  ProjectionUtils::Box3D box;

  if (!cloud || cloud->empty() || cluster.indices.empty()) {
    return box;
  }

  const auto & p_box = ProjectionUtils::getParams();
  std::vector<double> z_vals;
  z_vals.reserve(cluster.indices.size());
  for (int idx : cluster.indices) {
    z_vals.push_back(static_cast<double>(cloud->points[idx].z));
  }
  double z_lo, z_hi;
  getPercentileBounds(z_vals, p_box.z_extent_percentile_low, p_box.z_extent_percentile_high, z_lo, z_hi);
  box.center.z() = static_cast<float>(0.5 * (z_lo + z_hi));
  box.size.z() = static_cast<float>(std::max(0.0, z_hi - z_lo));

  std::vector<int> orientation_indices = cluster.indices;

  SearchResult fit_result = computeSearchBasedFit(*cloud, orientation_indices);

  if (fit_result.ok) {
    double ar =
      static_cast<double>(fit_result.len) / std::max(static_cast<double>(fit_result.wid), 0.1);

    Eigen::Vector4f centroid_temp;
    pcl::compute3DCentroid(*cloud, orientation_indices, centroid_temp);
    double yaw_los = std::atan2(centroid_temp.y(), centroid_temp.x());

    double final_yaw;

    if (ar < p_box.ar_front_view_threshold) {
      final_yaw = yaw_los;
    } else {
      double yaw0 = normalizeAngle(fit_result.yaw);
      double yaw1 = normalizeAngle(fit_result.yaw + M_PI);

      double diff0 = angleDiff(yaw0, yaw_los);
      double diff1 = angleDiff(yaw1, yaw_los);

      final_yaw = (diff1 < diff0) ? yaw1 : yaw0;
    }

    Eigen::Vector2f box_center_xy;
    Eigen::Vector2f box_size_xy;

    recomputeExtentsInYaw(*cloud, cluster.indices, final_yaw, fit_result.center_xy, box_center_xy, box_size_xy);

    box.center.x() = box_center_xy.x();
    box.center.y() = box_center_xy.y();
    box.size.x() = box_size_xy.x();
    box.size.y() = box_size_xy.y();
    box.yaw = final_yaw;

  } else {
    std::vector<double> x_vals, y_vals;
    x_vals.reserve(cluster.indices.size());
    y_vals.reserve(cluster.indices.size());
    for (int idx : cluster.indices) {
      const auto & pt = cloud->points[idx];
      x_vals.push_back(static_cast<double>(pt.x));
      y_vals.push_back(static_cast<double>(pt.y));
    }
    double x_lo, x_hi, y_lo, y_hi;
    getPercentileBounds(x_vals, p_box.xy_extent_percentile_low, p_box.xy_extent_percentile_high, x_lo, x_hi);
    getPercentileBounds(y_vals, p_box.xy_extent_percentile_low, p_box.xy_extent_percentile_high, y_lo, y_hi);
    box.center.x() = static_cast<float>(0.5 * (x_lo + x_hi));
    box.center.y() = static_cast<float>(0.5 * (y_lo + y_hi));
    box.size.x() = static_cast<float>(std::max(0.0, x_hi - x_lo));
    box.size.y() = static_cast<float>(std::max(0.0, y_hi - y_lo));
    box.yaw = 0.0;
  }

  return box;
}

std::vector<ProjectionUtils::ClusterStats> computeClusterStatsImpl(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices)
{
  std::vector<ProjectionUtils::ClusterStats> stats;
  stats.reserve(cluster_indices.size());

  const auto & p_ext = ProjectionUtils::getParams();
  for (const auto & c : cluster_indices) {
    ProjectionUtils::ClusterStats s;
    s.min_x = s.min_y = s.min_z = std::numeric_limits<float>::max();
    s.max_x = s.max_y = s.max_z = std::numeric_limits<float>::lowest();
    s.num_points = static_cast<int>(c.indices.size());

    if (c.indices.empty()) {
      stats.push_back(s);
      continue;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, c.indices, centroid);
    s.centroid = centroid;

    std::vector<double> x_vals, y_vals, z_vals;
    x_vals.reserve(c.indices.size());
    y_vals.reserve(c.indices.size());
    z_vals.reserve(c.indices.size());
    for (int idx : c.indices) {
      const auto & pt = cloud->points[idx];
      x_vals.push_back(static_cast<double>(pt.x));
      y_vals.push_back(static_cast<double>(pt.y));
      z_vals.push_back(static_cast<double>(pt.z));
    }
    double x_lo, x_hi, y_lo, y_hi, z_lo, z_hi;
    getPercentileBounds(x_vals, p_ext.xy_extent_percentile_low, p_ext.xy_extent_percentile_high, x_lo, x_hi);
    getPercentileBounds(y_vals, p_ext.xy_extent_percentile_low, p_ext.xy_extent_percentile_high, y_lo, y_hi);
    getPercentileBounds(z_vals, p_ext.z_extent_percentile_low, p_ext.z_extent_percentile_high, z_lo, z_hi);
    s.min_x = static_cast<float>(x_lo);
    s.max_x = static_cast<float>(x_hi);
    s.min_y = static_cast<float>(y_lo);
    s.max_y = static_cast<float>(y_hi);
    s.min_z = static_cast<float>(z_lo);
    s.max_z = static_cast<float>(z_hi);

    stats.push_back(s);
  }

  return stats;
}
}  // namespace

std::vector<ProjectionUtils::ClusterStats> ProjectionUtils::computeClusterStats(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices)
{
  return computeClusterStatsImpl(cloud, cluster_indices);
}

std::vector<ProjectionUtils::ClusterCandidate> ProjectionUtils::buildCandidates(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::vector<pcl::PointIndices> & cluster_indices)
{
  std::vector<ClusterCandidate> candidates;
  candidates.reserve(cluster_indices.size());
  std::vector<ClusterStats> stats = computeClusterStatsImpl(cloud, cluster_indices);
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    candidates.push_back(ClusterCandidate{cluster_indices[i], std::move(stats[i]), std::nullopt});
  }
  return candidates;
}

std::vector<pcl::PointIndices> ProjectionUtils::extractIndices(const std::vector<ClusterCandidate> & candidates)
{
  std::vector<pcl::PointIndices> indices;
  indices.reserve(candidates.size());
  for (const auto & c : candidates) {
    indices.push_back(c.indices);
  }
  return indices;
}

std::vector<ProjectionUtils::Box3D> ProjectionUtils::computeClusterBoxes(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices)
{
  std::vector<Box3D> boxes;
  boxes.reserve(cluster_indices.size());
  for (const auto & c : cluster_indices) {
    boxes.push_back(computeClusterBox(cloud, c));
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

Eigen::Matrix<double, 3, 4> ProjectionUtils::buildLidarToImageMatrix(
  const geometry_msgs::msg::TransformStamped & transform,
  const std::array<double, 12> & projection_matrix)
{
  const Eigen::Matrix4d T = transformToMatrix4d(transform);
  const Eigen::Matrix<double, 3, 4> P = arrayToProjection3x4(projection_matrix);
  return P * T;
}

std::optional<cv::Point2d> ProjectionUtils::projectLidarToCamera(
  const Eigen::Matrix<double, 3, 4> & lidar_to_image, const pcl::PointXYZ & pt)
{
  Eigen::Vector4d lidar_pt(pt.x, pt.y, pt.z, 1.0);
  Eigen::Vector3d projected = lidar_to_image * lidar_pt;
  if (projected.z() <= 0.0 ||
      projected.z() < ProjectionUtils::getParams().min_camera_z_distance) {
    return std::nullopt;
  }
  cv::Point2d proj_pt;
  proj_pt.x = projected.x() / projected.z();
  proj_pt.y = projected.y() / projected.z();
  return proj_pt;
}

void ProjectionUtils::euclideanClusterExtraction(
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
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minClusterSize);
  ec.setMaxClusterSize(maxClusterSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
}

void ProjectionUtils::adaptiveEuclideanClusterExtraction(
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

void ProjectionUtils::assignClusterColors(
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

void ProjectionUtils::mergeClusters(
  std::vector<pcl::PointIndices> & cluster_indices,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::vector<ClusterStats> & stats,
  double mergeTolerance)
{
  if (cloud->empty() || cluster_indices.empty() || stats.size() != cluster_indices.size()) return;

  std::vector<bool> merged(cluster_indices.size(), false);

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (merged[i]) continue;

    const Eigen::Vector4f & centroid_i = stats[i].centroid;

    for (size_t j = i + 1; j < cluster_indices.size(); ++j) {
      if (merged[j]) continue;

      const Eigen::Vector4f & centroid_j = stats[j].centroid;

      double distance = (centroid_i - centroid_j).norm();

      if (distance < mergeTolerance) {
        std::vector<int> merged_indices = cluster_indices[i].indices;
        merged_indices.insert(
          merged_indices.end(), cluster_indices[j].indices.begin(), cluster_indices[j].indices.end());
        cluster_indices[i].indices = std::move(merged_indices);
        merged[j] = true;
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

void ProjectionUtils::filterClustersByPhysicsConstraints(
  const std::vector<ClusterStats> & stats,
  std::vector<pcl::PointIndices> & cluster_indices,
  double max_distance,
  int min_points,
  float min_height,
  int min_points_default,
  int min_points_far,
  int min_points_medium,
  int min_points_large,
  double distance_threshold_far,
  double distance_threshold_medium,
  float volume_threshold_large,
  float min_density,
  float max_density,
  float max_dimension,
  float max_aspect_ratio)
{
  // Hardcoded thresholds (too specific to be configurable)
  constexpr float kMinDimensionForAspect = 0.05f;  // Minimum dimension to check aspect ratio (m)
  constexpr float kVolumeThresholdDensity = 0.01f;  // Minimum volume to check density (m³)

  if (stats.size() != cluster_indices.size() || cluster_indices.empty()) {
    return;
  }

  std::vector<pcl::PointIndices> kept_clusters;
  kept_clusters.reserve(cluster_indices.size());

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    const auto & s = stats[i];

    // Compute basic dimensions
    float width_x = s.max_x - s.min_x;
    float width_y = s.max_y - s.min_y;
    float height = s.max_z - s.min_z;
    float volume = width_x * width_y * height;

    // Distance from sensor
    double distance =
      std::sqrt(s.centroid.x() * s.centroid.x() + s.centroid.y() * s.centroid.y() + s.centroid.z() * s.centroid.z());

    // === IMPROVED FILTERING LOGIC ===

    // Filter 1: Minimum viable object
    if (s.num_points < min_points || height < min_height) {
      continue;
    }

    // Filter 2: Distance-adaptive point threshold
    int min_points_threshold = min_points_default;
    if (distance > distance_threshold_far) {
      min_points_threshold = min_points_far;
    } else if (distance > distance_threshold_medium) {
      min_points_threshold = min_points_medium;
    } else if (volume > volume_threshold_large) {
      min_points_threshold = min_points_large;
    }

    if (s.num_points < min_points_threshold) {
      continue;
    }

    // Filter 3: Density check (avoid too sparse or too dense)
    if (volume > kVolumeThresholdDensity) {
      float density = s.num_points / volume;
      if (density < min_density || density > max_density) {
        continue;
      }
    }

    // Filter 4: Geometric plausibility
    float max_dim = std::max({width_x, width_y, height});
    if (max_dim > max_dimension) {
      continue;
    }

    float min_dim = std::min({width_x, width_y, height});
    if (min_dim > kMinDimensionForAspect && max_dim / min_dim > max_aspect_ratio) {
      continue;  // Unrealistic aspect ratio
    }

    // Filter 5: Distance cutoff
    if (distance > max_distance) {
      continue;
    }

    // Passed all filters
    kept_clusters.push_back(cluster_indices[i]);
  }

  cluster_indices = std::move(kept_clusters);
}

void ProjectionUtils::filterCandidatesByPhysicsConstraints(
  std::vector<ClusterCandidate> & candidates,
  double max_distance,
  int min_points,
  float min_height,
  int min_points_default,
  int min_points_far,
  int min_points_medium,
  int min_points_large,
  double distance_threshold_far,
  double distance_threshold_medium,
  float volume_threshold_large,
  float min_density,
  float max_density,
  float max_dimension,
  float max_aspect_ratio)
{
  constexpr float kMinDimensionForAspect = 0.05f;
  constexpr float kVolumeThresholdDensity = 0.01f;

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

    if (s.num_points < min_points || height < min_height) continue;

    int min_points_threshold = min_points_default;
    if (distance > distance_threshold_far) {
      min_points_threshold = min_points_far;
    } else if (distance > distance_threshold_medium) {
      min_points_threshold = min_points_medium;
    } else if (volume > volume_threshold_large) {
      min_points_threshold = min_points_large;
    }
    if (s.num_points < min_points_threshold) continue;

    if (volume > kVolumeThresholdDensity) {
      float density = s.num_points / volume;
      if (density < min_density || density > max_density) continue;
    }

    float max_dim = std::max({width_x, width_y, height});
    if (max_dim > max_dimension) continue;

    float min_dim = std::min({width_x, width_y, height});
    if (min_dim > kMinDimensionForAspect && max_dim / min_dim > max_aspect_ratio) continue;

    if (distance > max_distance) continue;

    kept.push_back(std::move(cand));
  }
  candidates = std::move(kept);
}

// Class-specific geometry/quality limits for class-aware filtering (after IoU).
// Unknown classes use the default_max_* arguments passed to the filter.
struct ClassConstraints
{
  float max_dimension;
  float min_height;
  float max_aspect_ratio;
};

static ClassConstraints getClassConstraints(
  const std::string & class_id,
  float default_max_dimension,
  float default_min_height,
  float default_max_aspect_ratio)
{
  if (class_id == "person") {
    return {0.7f, 0.2f, 8.0f};  // Pedestrian: ~0.5 m typical, allow some margin
  }
  if (class_id == "car") {
    return {5.0f, 0.3f, 10.0f};
  }
  if (class_id == "truck" || class_id == "bus") {
    return {12.0f, 0.5f, 12.0f};
  }
  if (class_id == "bicycle" || class_id == "bike") {
    return {2.2f, 0.2f, 10.0f};
  }
  if (class_id == "motorcycle") {
    return {2.5f, 0.25f, 10.0f};
  }
  return {default_max_dimension, default_min_height, default_max_aspect_ratio};
}

static std::string getClassIdFromMatch(
  const ProjectionUtils::ClusterDetectionMatch & match,
  const vision_msgs::msg::Detection2DArray & detections)
{
  if (match.det_idx < 0 || static_cast<size_t>(match.det_idx) >= detections.detections.size()) {
    return "";
  }
  const auto & d = detections.detections[static_cast<size_t>(match.det_idx)];
  if (d.results.empty()) {
    return "";
  }
  return d.results[0].hypothesis.class_id;
}

void ProjectionUtils::filterCandidatesByClassAwareConstraints(
  std::vector<ClusterCandidate> & candidates,
  const vision_msgs::msg::Detection2DArray & detections,
  double max_distance,
  int min_points,
  float min_height,
  int min_points_default,
  int min_points_far,
  int min_points_medium,
  int min_points_large,
  double distance_threshold_far,
  double distance_threshold_medium,
  float volume_threshold_large,
  float min_density,
  float max_density,
  float default_max_dimension,
  float default_max_aspect_ratio)
{
  constexpr float kMinDimensionForAspect = 0.05f;
  constexpr float kVolumeThresholdDensity = 0.01f;

  std::vector<ClusterCandidate> kept;
  kept.reserve(candidates.size());
  for (auto & cand : candidates) {
    const auto & s = cand.stats;
    std::string class_id;
    if (cand.match.has_value()) {
      class_id = getClassIdFromMatch(cand.match.value(), detections);
    }
    const ClassConstraints cc =
      getClassConstraints(class_id, default_max_dimension, min_height, default_max_aspect_ratio);

    float width_x = s.max_x - s.min_x;
    float width_y = s.max_y - s.min_y;
    float height = s.max_z - s.min_z;
    float volume = width_x * width_y * height;
    double distance =
      std::sqrt(s.centroid.x() * s.centroid.x() + s.centroid.y() * s.centroid.y() + s.centroid.z() * s.centroid.z());

    if (s.num_points < min_points || height < cc.min_height) continue;

    int min_points_threshold = min_points_default;
    if (distance > distance_threshold_far) {
      min_points_threshold = min_points_far;
    } else if (distance > distance_threshold_medium) {
      min_points_threshold = min_points_medium;
    } else if (volume > volume_threshold_large) {
      min_points_threshold = min_points_large;
    }
    if (s.num_points < min_points_threshold) continue;

    if (volume > kVolumeThresholdDensity) {
      float density = s.num_points / volume;
      if (density < min_density || density > max_density) continue;
    }

    float max_dim = std::max({width_x, width_y, height});
    if (max_dim > cc.max_dimension) continue;
    float min_dim = std::min({width_x, width_y, height});
    if (min_dim > kMinDimensionForAspect && max_dim / min_dim > cc.max_aspect_ratio) continue;

    if (distance > max_distance) continue;

    kept.push_back(std::move(cand));
  }
  candidates = std::move(kept);
}

bool ProjectionUtils::computeClusterCentroid(
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
/**
 * @brief Project AABB 8 corners from ClusterStats to a 2D bounding rect, clipped to image bounds.
 * Much faster than sampling N cluster points: only 8 projections, no cloud access needed.
 */
std::optional<cv::Rect2d> projectAABBRect(
  const ProjectionUtils::ClusterStats & stats,
  const Eigen::Matrix<double, 3, 4> & lidar_to_image,
  double image_w, double image_h)
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
        auto uv = ProjectionUtils::projectLidarToCamera(lidar_to_image, pt);
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

void ProjectionUtils::assignCandidatesToDetectionsByIOU(
  std::vector<ClusterCandidate> & candidates,
  const vision_msgs::msg::Detection2DArray & detections,
  const geometry_msgs::msg::TransformStamped & transform,
  const std::array<double, 12> & projection_matrix,
  float object_detection_confidence)
{
  if (candidates.empty()) return;

  if (detections.detections.empty()) {
    candidates.clear();
    return;
  }

  const Eigen::Matrix<double, 3, 4> lidar_to_image =
    ProjectionUtils::buildLidarToImageMatrix(transform, projection_matrix);
  const double iw = static_cast<double>(image_width_);
  const double ih = static_cast<double>(image_height_);
  const double min_iou = ProjectionUtils::getParams().min_iou_threshold;

  struct Pair
  {
    size_t cand_idx;
    int det_idx;
    double iou;
  };
  std::vector<Pair> pairs;

  for (size_t c = 0; c < candidates.size(); ++c) {
    auto cluster_rect = projectAABBRect(candidates[c].stats, lidar_to_image, iw, ih);
    if (!cluster_rect) continue;

    for (size_t d = 0; d < detections.detections.size(); ++d) {
      const auto & det = detections.detections[d];
      if (!det.results.empty() && det.results[0].hypothesis.score < object_detection_confidence) {
        continue;
      }
      const auto & b = det.bbox;
      const cv::Rect2d det_rect(
        b.center.position.x - b.size_x / 2.0,
        b.center.position.y - b.size_y / 2.0,
        b.size_x,
        b.size_y);
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

  std::sort(assignments.begin(), assignments.end(),
            [](const Pair & a, const Pair & b) { return a.cand_idx < b.cand_idx; });

  std::vector<ClusterCandidate> kept;
  kept.reserve(assignments.size());
  for (const auto & a : assignments) {
    ClusterCandidate cand = std::move(candidates[a.cand_idx]);
    cand.match = ClusterDetectionMatch{a.cand_idx, a.det_idx, a.iou};
    kept.push_back(std::move(cand));
  }
  candidates = std::move(kept);
}

visualization_msgs::msg::MarkerArray ProjectionUtils::computeBoundingBox(
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

    const auto & p_viz = ProjectionUtils::getParams();
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = p_viz.marker_alpha;

    bbox_marker.lifetime = rclcpp::Duration::from_seconds(p_viz.marker_lifetime_s);

    marker_array.markers.push_back(bbox_marker);
  }

  return marker_array;
}

vision_msgs::msg::Detection3DArray ProjectionUtils::compute3DDetection(
  const std::vector<Box3D> & boxes,
  const std::vector<ClusterCandidate> & candidates,
  const std_msgs::msg::Header & header,
  const vision_msgs::msg::Detection2DArray & detections)
{
  vision_msgs::msg::Detection3DArray det_arr;
  det_arr.header = header;

  if (boxes.size() != candidates.size()) return det_arr;

  const auto & params = ProjectionUtils::getParams();

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
          hypo.hypothesis.score =
            params.detection_score_weight * det_score + params.iou_score_weight * m.iou;
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
