#include "orientation_utils.hpp"
#include <pcl/common/centroid.h>
#include <cmath>
#include <limits>
#include <algorithm>

namespace OrientationUtils {

double normalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double angleDiff(double a, double b) {
  double d = normalizeAngle(a - b);
  return std::abs(d);
}

std::vector<Eigen::Vector2f> samplePointsXY(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<int>& indices,
    size_t desired_count) {
  std::vector<Eigen::Vector2f> pts;
  if (indices.empty()) return pts;

  size_t step = 1;
  if (indices.size() > desired_count) {
    step = indices.size() / desired_count;
    if (step == 0) step = 1; // safety
  }

  pts.reserve((indices.size() + step - 1) / step);
  for (size_t i = 0; i < indices.size(); i += step) {
    const auto& p = cloud.points[indices[i]];
    pts.emplace_back(p.x, p.y);
  }
  return pts;
}

// --- EDGE-ALIGNMENT FIT (The "Anti-Diagonal" Fix) ---
// Instead of minimizing Area (which fails on L-shapes), we minimize
// the distance of points to the bounding box edges.
SearchResult computeSearchBasedFit(const pcl::PointCloud<pcl::PointXYZ>& pts,
                                   const std::vector<int>& indices) {
  SearchResult r;
  if (indices.size() < 3) { r.ok = false; return r; }

  // 1. Select a subset of points for speed (Downsampling)
  // We don't need 500 points to find the angle. ~50-100 is plenty.
  // This replaces the Convex Hull optimization with something safer for L-shapes.
  auto search_points = samplePointsXY(pts, indices);
  if (search_points.size() < 3) { r.ok = false; return r; }

  double min_energy = std::numeric_limits<double>::max();
  double best_theta = 0.0;

  // 2. Pre-allocate rotated cache once to avoid allocations per angle
  std::vector<std::pair<double, double>> rotated_cache;
  rotated_cache.resize(search_points.size());

  // 3. Define the Scoring Function: Edge Energy
  // We want points to be CLOSE to the min/max bounds (the walls).
  // Energy = Sum of squared distances to the nearest wall.
  auto calculateEdgeEnergy = [&](double theta) -> double {
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    
    // First pass: Find the bounding box for this rotation
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (size_t i = 0; i < search_points.size(); ++i) {
      const auto& p = search_points[i];
      double x_rot = static_cast<double>(p.x()) * cos_t + static_cast<double>(p.y()) * sin_t;
      double y_rot = -static_cast<double>(p.x()) * sin_t + static_cast<double>(p.y()) * cos_t;
      
      rotated_cache[i] = {x_rot, y_rot};

      if (x_rot < min_x) min_x = x_rot;
      if (x_rot > max_x) max_x = x_rot;
      if (y_rot < min_y) min_y = y_rot;
      if (y_rot > max_y) max_y = y_rot;
    }

    // Second pass: Calculate how "stuck" the points are to the walls
    double energy = 0.0;
    for (const auto& pr : rotated_cache) {
        // Distance to X walls
        double dx = std::min(std::abs(pr.first - min_x), std::abs(pr.first - max_x));
        // Distance to Y walls
        double dy = std::min(std::abs(pr.second - min_y), std::abs(pr.second - max_y));
        
        // Distance to NEAREST wall
        double d = std::min(dx, dy);
        
        // Sum of distances (L1 norm is robust)
        energy += d; 
    }
    
    // Normalize energy by number of points so it's invariant to sample size
    return energy / static_cast<double>(search_points.size());
  };

  // 4. COARSE SEARCH (0 to 90 degrees, step 10 degrees)
  double coarse_step = 10.0 * M_PI / 180.0;
  for (double theta = 0.0; theta < M_PI_2; theta += coarse_step) {
    double energy = calculateEdgeEnergy(theta);
    if (energy < min_energy) {
      min_energy = energy;
      best_theta = theta;
    }
  }

  // 5. FINE SEARCH (Range +/- 5 degrees, step 2 degrees)
  double fine_range = 5.0 * M_PI / 180.0;
  double fine_step = 2.0 * M_PI / 180.0; // 2 degree precision is usually enough
  
  double start_theta = std::max(0.0, best_theta - fine_range);
  double end_theta   = std::min(M_PI_2, best_theta + fine_range);

  for (double theta = start_theta; theta <= end_theta; theta += fine_step) {
    double energy = calculateEdgeEnergy(theta);
    if (energy < min_energy) {
      min_energy = energy;
      best_theta = theta;
    }
  }
  
  // 6. FINAL RE-CALCULATION (Using ALL points, not just subset)
  // We found the best angle using the subset, now fit the box to everything.
  std::vector<Eigen::Vector2f> all_points_2d;
  all_points_2d.reserve(indices.size());
  for (int idx : indices) {
      const auto& p = pts.points[idx];
      all_points_2d.emplace_back(p.x, p.y);
  }

  double cos_t = std::cos(best_theta);
  double sin_t = std::sin(best_theta);
  
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& p : all_points_2d) {
    double x_rot = static_cast<double>(p.x()) * cos_t + static_cast<double>(p.y()) * sin_t;
    double y_rot = -static_cast<double>(p.x()) * sin_t + static_cast<double>(p.y()) * cos_t;
    if (x_rot < min_x) min_x = x_rot;
    if (x_rot > max_x) max_x = x_rot;
    if (y_rot < min_y) min_y = y_rot;
    if (y_rot > max_y) max_y = y_rot;
  }

  // Transform center back to global
  double cx_rot = 0.5 * (min_x + max_x);
  double cy_rot = 0.5 * (min_y + max_y);

  double cx = cx_rot * cos_t - cy_rot * sin_t;
  double cy = cx_rot * sin_t + cy_rot * cos_t;

  r.center_xy = Eigen::Vector2f(static_cast<float>(cx), static_cast<float>(cy));
  
  double d1 = max_x - min_x;
  double d2 = max_y - min_y;

  // Standardize: Length >= Width
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

OrientationResult computeClusterOrientation(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<int>& orientation_indices) {
  OrientationResult result;
  
  // Perform Fit using Search-Based Fit
  SearchResult fit_result = computeSearchBasedFit(cloud, orientation_indices);

  if (fit_result.ok) {
    // Aspect Ratio Check
    double ar = static_cast<double>(fit_result.len) /
                std::max(static_cast<double>(fit_result.wid), 0.1);

    // Calculate LOS Yaw
    Eigen::Vector4f centroid_temp;
    pcl::compute3DCentroid(cloud, orientation_indices, centroid_temp);
    double yaw_los = std::atan2(centroid_temp.y(), centroid_temp.x());

    double final_yaw;

    // Square/Wall Check
    if (ar < kARFrontViewThreshold) {
      final_yaw = yaw_los;
    } else {
      // Elongated: Resolve 180 ambiguity
      double yaw0 = normalizeAngle(fit_result.yaw);
      double yaw1 = normalizeAngle(fit_result.yaw + M_PI);

      double diff0 = angleDiff(yaw0, yaw_los);
      double diff1 = angleDiff(yaw1, yaw_los);

      final_yaw = (diff1 < diff0) ? yaw1 : yaw0;
    }

    result.center_xy = fit_result.center_xy;
    result.len = fit_result.len;
    result.wid = fit_result.wid;
    result.yaw = final_yaw;
    result.ok = true;
  } else {
    result.ok = false;
  }

  return result;
}

} // namespace OrientationUtils

