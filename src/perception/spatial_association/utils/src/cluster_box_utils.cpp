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
#include <limits>
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

}  // namespace cluster_box
