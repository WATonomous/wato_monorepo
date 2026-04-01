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

#include "utils/cluster_box_utils.hpp"

#include <pcl/common/centroid.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cluster_box
{

static void getPercentileBounds(
  std::vector<double> & data, double low_pct, double high_pct, double & out_low, double & out_high)
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

static double normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double angleDiff(double a, double b)
{
  double d = normalizeAngle(a - b);
  return std::abs(d);
}

static std::vector<Eigen::Vector2f> samplePointsXY(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::vector<int> & indices, size_t desired_count)
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

static SearchResult computeSearchBasedFit(const pcl::PointCloud<pcl::PointXYZ> & pts, const std::vector<int> & indices)
{
  const auto & p = projection_utils::getParams();
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
  const double coarse_step = p.orientation_coarse_step_multiplier * step_rad;
  for (double theta = 0.0; theta < M_PI_2; theta += coarse_step) {
    double energy = calculateEdgeEnergy(theta);
    if (energy < min_energy) {
      min_energy = energy;
      best_theta = theta;
    }
  }

  const double fine_range = p.orientation_fine_range_multiplier * step_rad;
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

static void recomputeExtentsInYaw(
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

  const auto & p_rej = projection_utils::getParams();
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

projection_utils::Box3D computeClusterBox(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::PointIndices & cluster)
{
  projection_utils::Box3D box;

  if (!cloud || cloud->empty() || cluster.indices.empty()) {
    return box;
  }

  const auto & p_box = projection_utils::getParams();
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
    double ar = static_cast<double>(fit_result.len) / std::max(static_cast<double>(fit_result.wid), 0.1);

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

projection_utils::ClusterStats computeSingleClusterStats(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::PointIndices & indices)
{
  projection_utils::ClusterStats s;
  s.min_x = s.min_y = s.min_z = std::numeric_limits<float>::max();
  s.max_x = s.max_y = s.max_z = std::numeric_limits<float>::lowest();
  s.num_points = static_cast<int>(indices.indices.size());

  if (indices.indices.empty()) {
    return s;
  }

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, indices.indices, centroid);
  s.centroid = centroid;

  const auto & p_ext = projection_utils::getParams();
  std::vector<double> x_vals, y_vals, z_vals;
  x_vals.reserve(indices.indices.size());
  y_vals.reserve(indices.indices.size());
  z_vals.reserve(indices.indices.size());
  for (int idx : indices.indices) {
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

  return s;
}

std::vector<projection_utils::ClusterStats> computeClusterStatsBatch(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::vector<pcl::PointIndices> & cluster_indices)
{
  std::vector<projection_utils::ClusterStats> stats;
  stats.reserve(cluster_indices.size());

  for (const auto & c : cluster_indices) {
    stats.push_back(computeSingleClusterStats(cloud, c));
  }

  return stats;
}

/**
 * @brief L-shaped fitting for vehicle-like clusters.
 *
 * For each candidate yaw angle, projects points to the rotated frame and finds the closest
 * edge (left/right/front/back) for each point. The angle that minimizes total closest-edge
 * distance is the best L-shape orientation. This works well when LiDAR sees 1-2 sides of
 * a rectangular object.
 */
static SearchResult computeLShapedFit(const pcl::PointCloud<pcl::PointXYZ> & pts, const std::vector<int> & indices)
{
  const auto & p = projection_utils::getParams();
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

  const double pct_lo = p.xy_extent_percentile_low;
  const double pct_hi = p.xy_extent_percentile_high;

  double min_energy = std::numeric_limits<double>::max();
  double best_theta = 0.0;

  // L-shaped criterion: for each angle, compute how close points are to the nearest edge
  // of the bounding rectangle. Lower total = better L-fit (points cluster near edges).
  auto calculateLShapeEnergy = [&](double theta) -> double {
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x;
    double max_y = max_x;

    std::vector<std::pair<double, double>> rotated(search_points.size());
    for (size_t i = 0; i < search_points.size(); ++i) {
      const auto & pt = search_points[i];
      double x_rot = static_cast<double>(pt.x()) * cos_t + static_cast<double>(pt.y()) * sin_t;
      double y_rot = -static_cast<double>(pt.x()) * sin_t + static_cast<double>(pt.y()) * cos_t;
      rotated[i] = {x_rot, y_rot};
      min_x = std::min(min_x, x_rot);
      max_x = std::max(max_x, x_rot);
      min_y = std::min(min_y, y_rot);
      max_y = std::max(max_y, y_rot);
    }

    // L-shape energy: variance of the closest-edge distances (low variance = L-shaped).
    double sum_d = 0.0;
    double sum_d2 = 0.0;
    for (const auto & pr : rotated) {
      double dx = std::min(std::abs(pr.first - min_x), std::abs(pr.first - max_x));
      double dy = std::min(std::abs(pr.second - min_y), std::abs(pr.second - max_y));
      double d = std::min(dx, dy);
      sum_d += d;
      sum_d2 += d * d;
    }
    double n = static_cast<double>(rotated.size());
    double mean = sum_d / n;
    double variance = sum_d2 / n - mean * mean;

    // Combined: mean closeness + variance penalty (L-shapes have low mean AND low variance).
    return mean + p.l_shape_energy_variance_weight * variance;
  };

  // Coarse search: 0-90 degrees.
  const double step_rad = p.orientation_search_step_degrees * M_PI / 180.0;
  const double coarse_step = p.orientation_coarse_step_multiplier * step_rad;
  for (double theta = 0.0; theta < M_PI_2; theta += coarse_step) {
    double energy = calculateLShapeEnergy(theta);
    if (energy < min_energy) {
      min_energy = energy;
      best_theta = theta;
    }
  }

  // Fine search around the best coarse angle.
  double start = std::max(0.0, best_theta - p.orientation_fine_range_multiplier * step_rad);
  double end = std::min(M_PI_2, best_theta + p.orientation_fine_range_multiplier * step_rad);
  for (double theta = start; theta <= end; theta += step_rad) {
    double energy = calculateLShapeEnergy(theta);
    if (energy < min_energy) {
      min_energy = energy;
      best_theta = theta;
    }
  }

  // Compute final extents using all points.
  std::vector<double> x_rot_all, y_rot_all;
  x_rot_all.reserve(indices.size());
  y_rot_all.reserve(indices.size());
  double cos_t = std::cos(best_theta);
  double sin_t = std::sin(best_theta);
  for (int idx : indices) {
    const auto & pt = pts.points[idx];
    x_rot_all.push_back(static_cast<double>(pt.x) * cos_t + static_cast<double>(pt.y) * sin_t);
    y_rot_all.push_back(-static_cast<double>(pt.x) * sin_t + static_cast<double>(pt.y) * cos_t);
  }

  double min_x, max_x, min_y, max_y;
  getPercentileBounds(x_rot_all, pct_lo, pct_hi, min_x, max_x);
  getPercentileBounds(y_rot_all, pct_lo, pct_hi, min_y, max_y);

  double cx_rot = 0.5 * (min_x + max_x);
  double cy_rot = 0.5 * (min_y + max_y);
  r.center_xy = Eigen::Vector2f(
    static_cast<float>(cx_rot * cos_t - cy_rot * sin_t), static_cast<float>(cx_rot * sin_t + cy_rot * cos_t));

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

projection_utils::Box3D computeClusterBoxWithClassHint(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::PointIndices & cluster, const std::string & class_hint)
{
  const auto & params = projection_utils::getParams();
  const bool is_vehicle = (class_hint == "car" || class_hint == "truck" || class_hint == "bus");

  if (!is_vehicle || !params.use_l_shaped_fitting || !cloud || cluster.indices.size() < 5u) {
    return computeClusterBox(cloud, cluster);
  }

  // Run both search-based and L-shaped fitting, pick the one with lower energy.
  projection_utils::Box3D box;
  const auto & p_box = projection_utils::getParams();

  // Compute Z extents (shared).
  std::vector<double> z_vals;
  z_vals.reserve(cluster.indices.size());
  for (int idx : cluster.indices) {
    z_vals.push_back(static_cast<double>(cloud->points[idx].z));
  }
  double z_lo, z_hi;
  getPercentileBounds(z_vals, p_box.z_extent_percentile_low, p_box.z_extent_percentile_high, z_lo, z_hi);
  box.center.z() = static_cast<float>(0.5 * (z_lo + z_hi));
  box.size.z() = static_cast<float>(std::max(0.0, z_hi - z_lo));

  SearchResult search_fit = computeSearchBasedFit(*cloud, cluster.indices);
  SearchResult l_fit = computeLShapedFit(*cloud, cluster.indices);

  // Pick L-shaped if it produced a valid result.
  // L-shaped fitting is preferred for vehicles because it handles partial visibility better.
  SearchResult & chosen = (l_fit.ok) ? l_fit : search_fit;
  if (!chosen.ok) {
    // Fallback to AABB.
    return computeClusterBox(cloud, cluster);
  }

  // Resolve yaw ambiguity using line-of-sight.
  double ar = static_cast<double>(chosen.len) / std::max(static_cast<double>(chosen.wid), 0.1);
  Eigen::Vector4f centroid_temp;
  pcl::compute3DCentroid(*cloud, cluster.indices, centroid_temp);
  double yaw_los = std::atan2(centroid_temp.y(), centroid_temp.x());

  double final_yaw;
  if (ar < p_box.ar_front_view_threshold) {
    final_yaw = yaw_los;
  } else {
    double yaw0 = normalizeAngle(chosen.yaw);
    double yaw1 = normalizeAngle(chosen.yaw + M_PI);
    final_yaw = (angleDiff(yaw1, yaw_los) < angleDiff(yaw0, yaw_los)) ? yaw1 : yaw0;
  }

  Eigen::Vector2f box_center_xy, box_size_xy;
  recomputeExtentsInYaw(*cloud, cluster.indices, final_yaw, chosen.center_xy, box_center_xy, box_size_xy);

  box.center.x() = box_center_xy.x();
  box.center.y() = box_center_xy.y();
  box.size.x() = box_size_xy.x();
  box.size.y() = box_size_xy.y();
  box.yaw = final_yaw;

  return box;
}

void applyClassAwareBoxExtension(projection_utils::Box3D & box, const std::string & class_hint)
{
  const auto & params = projection_utils::getParams();
  if (!params.enable_size_prior) return;
  if (class_hint.empty()) return;
  auto it = params.size_priors.find(class_hint);
  if (it == params.size_priors.end()) return;

  const auto & prior = it->second;

  // Box convention: size.x = along yaw (length), size.y = perpendicular (width).
  const double target_length = 0.5 * (prior.min_length + prior.max_length);
  const double target_width = 0.5 * (prior.min_width + prior.max_width);

  const double cx = static_cast<double>(box.center.x());
  const double cy = static_cast<double>(box.center.y());
  const double los_angle = std::atan2(cy, cx);

  // How aligned is each box axis with the radial (ego-to-object) direction?
  // cos_rel ≈ 1: x-axis (yaw) points radially → x is depth, y is cross-range (well-observed).
  // cos_rel ≈ 0: y-axis points radially → y is depth, x is cross-range (well-observed).
  const double cos_rel = std::abs(std::cos(box.yaw - los_angle));

  // Only extend the axis that is predominantly radial (facing away from ego).
  // Threshold at cos(45°) ≈ 0.707: if the angle is ambiguous, don't extend — LiDAR sees
  // both sides partially and the fit is reasonable as-is.
  constexpr double kMinRadialAlignment = 0.707;  // cos(45°)

  const double cos_yaw = std::cos(box.yaw);
  const double sin_yaw = std::sin(box.yaw);

  if (cos_rel >= kMinRadialAlignment) {
    // x-axis (length) is radial → LiDAR can't see it well. Extend length only.
    if (static_cast<double>(box.size.x()) < target_length) {
      const double delta = target_length - static_cast<double>(box.size.x());
      const double dot = cos_yaw * cx + sin_yaw * cy;
      const double sign = (dot >= 0.0) ? 1.0 : -1.0;
      box.center.x() = static_cast<float>(cx + sign * cos_yaw * delta * 0.5);
      box.center.y() = static_cast<float>(cy + sign * sin_yaw * delta * 0.5);
      box.size.x() = static_cast<float>(target_length);
    }
  } else if (cos_rel <= (1.0 - kMinRadialAlignment)) {
    // y-axis (width) is radial → LiDAR can't see it well. Extend width only.
    if (static_cast<double>(box.size.y()) < target_width) {
      const double delta = target_width - static_cast<double>(box.size.y());
      const double dot_perp = -sin_yaw * cx + cos_yaw * cy;
      const double sign = (dot_perp >= 0.0) ? 1.0 : -1.0;
      box.center.x() = static_cast<float>(cx + sign * (-sin_yaw) * delta * 0.5);
      box.center.y() = static_cast<float>(cy + sign * cos_yaw * delta * 0.5);
      box.size.y() = static_cast<float>(target_width);
    }
  }
  // else: ambiguous viewing angle (near 45°) — don't extend, the fit is the best we have.
}

}  // namespace cluster_box
