#include "projection_utils.hpp"
#include <cmath>
#include <limits>
#include <array>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>
// Removed opencv header as we don't need convexHull anymore

// PRE-CLUSTER FILTERING
// ----------------------------------------------------------------------------------------------------------------------


namespace {
// Orientation and filtering thresholds
constexpr float kTallObjectHeightThreshold = 1.5f;
constexpr float kTallObjectCutBottomFraction = 0.4f;
constexpr float kShortObjectCutBottomMeters = 0.2f;

constexpr double kARFrontViewThreshold = 1.4; // front-wall vs elongated
constexpr size_t kOutlierRejectionPointCount = 100;
constexpr double kOutlierSigmaMultiplier = 3.5;

struct SearchResult {
  Eigen::Vector2f center_xy{0.f, 0.f};
  double yaw{0.0};
  float len{0.f};
  float wid{0.f};
  bool ok{false};
};

inline double normalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double angleDiff(double a, double b) {
  double d = normalizeAngle(a - b);
  return std::abs(d);
}

// Helper function to sample points for orientation search
std::vector<Eigen::Vector2f> samplePointsXY(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<int>& indices,
    size_t desired_count = 64) {
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

void recomputeExtentsInYaw(
    const pcl::PointCloud<pcl::PointXYZ>& pts,
    const std::vector<int>& indices,
    double yaw,
    const Eigen::Vector2f& base_center,  // initial center (from search-based fit)
    Eigen::Vector2f& out_center,
    Eigen::Vector2f& out_size) {
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  // 1. Project points to rotated frame (rotating around base_center)
  // We want to center the box on the DENSITY of points, not the geometric average of outliers
  std::vector<double> xs_rot;
  std::vector<double> ys_rot;
  xs_rot.reserve(indices.size());
  ys_rot.reserve(indices.size());

  double sum_xr = 0.0, sum_yr = 0.0;

  for (int idx : indices) {
    const auto& pt = pts.points[idx];
    double dx = pt.x - base_center.x();
    double dy = pt.y - base_center.y();

    double x_rot = dx * cos_yaw + dy * sin_yaw;
    double y_rot = -dx * sin_yaw + dy * cos_yaw;

    xs_rot.push_back(x_rot);
    ys_rot.push_back(y_rot);
    sum_xr += x_rot;
    sum_yr += y_rot;
  }

  double mean_xr = sum_xr / indices.size();
  double mean_yr = sum_yr / indices.size();

  // 2. Calculate Variance/StdDev
  double var_xr = 0.0, var_yr = 0.0;
  for (size_t i = 0; i < indices.size(); ++i) {
    var_xr += (xs_rot[i] - mean_xr) * (xs_rot[i] - mean_xr);
    var_yr += (ys_rot[i] - mean_yr) * (ys_rot[i] - mean_yr);
  }
  double std_xr = std::sqrt(var_xr / indices.size());
  double std_yr = std::sqrt(var_yr / indices.size());

  // 3. Find Bounds
  // For large point sets (all cluster points), don't apply outlier rejection to ensure full coverage.
  // For small point sets (filtered orientation points), apply mild outlier rejection.
  const bool apply_outlier_rejection = (indices.size() < kOutlierRejectionPointCount);
  const double sigma_mul = kOutlierSigmaMultiplier;

  double min_x_rot = std::numeric_limits<double>::infinity();
  double max_x_rot = -std::numeric_limits<double>::infinity();
  double min_y_rot = std::numeric_limits<double>::infinity();
  double max_y_rot = -std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < indices.size(); ++i) {
    double val_x = xs_rot[i];
    double val_y = ys_rot[i];

    // Only apply outlier rejection for small filtered point sets
    if (apply_outlier_rejection) {
      // Soft clamping: we still want the box to cover the car, but stop distant outliers
      if (val_x > mean_xr + sigma_mul * std_xr) val_x = mean_xr + sigma_mul * std_xr;
      if (val_x < mean_xr - sigma_mul * std_xr) val_x = mean_xr - sigma_mul * std_xr;
      if (val_y > mean_yr + sigma_mul * std_yr) val_y = mean_yr + sigma_mul * std_yr;
      if (val_y < mean_yr - sigma_mul * std_yr) val_y = mean_yr - sigma_mul * std_yr;
    }

    min_x_rot = std::min(min_x_rot, val_x);
    max_x_rot = std::max(max_x_rot, val_x);
    min_y_rot = std::min(min_y_rot, val_y);
    max_y_rot = std::max(max_y_rot, val_y);
  }

  // 4. Offset within the rotated frame
  double center_x_rot = 0.5 * (min_x_rot + max_x_rot);
  double center_y_rot = 0.5 * (min_y_rot + max_y_rot);

  // 5. Transform back to global frame
  double center_x = base_center.x() + center_x_rot * cos_yaw - center_y_rot * sin_yaw;
  double center_y = base_center.y() + center_x_rot * sin_yaw + center_y_rot * cos_yaw;

  out_center = Eigen::Vector2f(static_cast<float>(center_x), static_cast<float>(center_y));
  out_size = Eigen::Vector2f(static_cast<float>(max_x_rot - min_x_rot),
                             static_cast<float>(max_y_rot - min_y_rot));
}


ProjectionUtils::Box3D computeClusterBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const pcl::PointIndices& cluster) {
  ProjectionUtils::Box3D box;

  if (!cloud || cloud->empty() || cluster.indices.empty()) {
    return box;
  }

  // 1. Collect all points and compute initial z bounds
  std::vector<std::pair<float, int>> z_sorted_indices;
  z_sorted_indices.reserve(cluster.indices.size());

  float min_z_all = std::numeric_limits<float>::max();
  float max_z_all = std::numeric_limits<float>::lowest();

  for (int idx : cluster.indices) {
    const auto& pt = cloud->points[idx];
    if (pt.z < min_z_all) min_z_all = pt.z;
    if (pt.z > max_z_all) max_z_all = pt.z;
    z_sorted_indices.push_back({pt.z, idx});
  }

  const float height = max_z_all - min_z_all;

  // 2. Select Points for Orientation (z-sliced to avoid ground)
  std::vector<int> orientation_indices;
  float z_threshold;
  
  // High object threshold? Cut bottom fraction
  if (height > kTallObjectHeightThreshold) {
    z_threshold = min_z_all + (height * kTallObjectCutBottomFraction);
  } else {
    z_threshold = min_z_all + kShortObjectCutBottomMeters;
  }

  for (const auto& pair : z_sorted_indices) {
    if (pair.first >= z_threshold) orientation_indices.push_back(pair.second);
  }

  // Fallback: Top 50%
  if (orientation_indices.size() < 5 && cluster.indices.size() > 10) {
    std::sort(z_sorted_indices.begin(), z_sorted_indices.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; }); 
    orientation_indices.clear();
    size_t count_to_take = z_sorted_indices.size() / 2; 
    if (count_to_take < 5) count_to_take = z_sorted_indices.size();
    for (size_t i = 0; i < count_to_take; ++i) orientation_indices.push_back(z_sorted_indices[i].second);
  }
  if (orientation_indices.size() < 3) orientation_indices = cluster.indices;

  // 3. Compute filtered z bounds (exclude ground noise for height calculation)
  // Use the same z_threshold to filter ground points
  float min_z_filtered = std::numeric_limits<float>::max();
  float max_z_filtered = std::numeric_limits<float>::lowest();

  for (const auto& pair : z_sorted_indices) {
    if (pair.first >= z_threshold) {
      if (pair.first < min_z_filtered) min_z_filtered = pair.first;
      if (pair.first > max_z_filtered) max_z_filtered = pair.first;
    }
  }

  // If filtering removed too many points, use all points for z
  if (min_z_filtered == std::numeric_limits<float>::max()) {
    min_z_filtered = min_z_all;
    max_z_filtered = max_z_all;
  } else {
    // Ensure we have at least the orientation points' z range
    for (int idx : orientation_indices) {
      const auto& pt = cloud->points[idx];
      if (pt.z < min_z_filtered) min_z_filtered = pt.z;
      if (pt.z > max_z_filtered) max_z_filtered = pt.z;
    }
  }

  // Z-axis (height) - use filtered points to avoid ground noise
  box.center.z() = 0.5f * (min_z_filtered + max_z_filtered);
  box.size.z()   = std::max(0.0f, max_z_filtered - min_z_filtered);

  // 4. Perform Fit using Search-Based Fit (NOT cv::minAreaRect)
  SearchResult fit_result = computeSearchBasedFit(*cloud, orientation_indices);

  if (fit_result.ok) {
    // Aspect Ratio Check
    double ar = static_cast<double>(fit_result.len) /
                std::max(static_cast<double>(fit_result.wid), 0.1);

    // Calculate LOS Yaw
    Eigen::Vector4f centroid_temp;
    pcl::compute3DCentroid(*cloud, orientation_indices, centroid_temp);
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

    // Use fit_result directly (no need to recompute extents - ground points don't change XY footprint)
    box.center.x() = fit_result.center_xy.x();
    box.center.y() = fit_result.center_xy.y();
    box.size.x()   = fit_result.len;
    box.size.y()   = fit_result.wid;
    box.yaw        = final_yaw;

  } else {
    // Fallback: Axis Aligned
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    for (int idx : cluster.indices) {
      const auto& pt = cloud->points[idx];
      if (pt.x < min_x) min_x = pt.x;
      if (pt.x > max_x) max_x = pt.x;
      if (pt.y < min_y) min_y = pt.y;
      if (pt.y > max_y) max_y = pt.y;
    }

    box.center.x() = 0.5f * (min_x + max_x);
    box.center.y() = 0.5f * (min_y + max_y);
    box.size.x()   = std::max(0.0f, max_x - min_x);
    box.size.y()   = std::max(0.0f, max_y - min_y);
    box.yaw        = 0.0;
  }

  return box;
}
} // namespace

std::vector<ProjectionUtils::ClusterStats> ProjectionUtils::computeClusterStats(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices) {
  std::vector<ClusterStats> stats;
  stats.reserve(cluster_indices.size());

  for (const auto& c : cluster_indices) {
    ClusterStats s;
    s.min_x = s.min_y = s.min_z = std::numeric_limits<float>::max();
    s.max_x = s.max_y = s.max_z = std::numeric_limits<float>::lowest();
    s.num_points = static_cast<int>(c.indices.size());

    if (c.indices.empty()) {
      stats.push_back(s);
      continue;
    }

    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, c.indices, centroid);
    s.centroid = centroid;

    // Compute min/max bounds
    for (int idx : c.indices) {
      const auto& p = cloud->points[idx];
      if (p.x < s.min_x) s.min_x = p.x;
      if (p.x > s.max_x) s.max_x = p.x;
      if (p.y < s.min_y) s.min_y = p.y;
      if (p.y > s.max_y) s.max_y = p.y;
      if (p.z < s.min_z) s.min_z = p.z;
      if (p.z > s.max_z) s.max_z = p.z;
    }

    stats.push_back(s);
  }

  return stats;
}

std::vector<ProjectionUtils::Box3D> ProjectionUtils::computeClusterBoxes(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices) {
  std::vector<Box3D> boxes;
  boxes.reserve(cluster_indices.size());
  for (const auto& c : cluster_indices) {
    boxes.push_back(computeClusterBox(cloud, c));
  }
  return boxes;
}

std::optional<cv::Point2d> ProjectionUtils::projectLidarToCamera(
    const geometry_msgs::msg::TransformStamped& transform, const std::array<double, 12>& p,
    const pcl::PointXYZ& pt) {
  /*
      Projects a 3D lidar point onto a 2d camera image using the given extrinsic transformation and
     camera projection matrix

      Purpose: Returns 2d image coordinates as a cv::Point2d object
  */

  geometry_msgs::msg::PointStamped lidar_point;
  lidar_point.point.x = pt.x;
  lidar_point.point.y = pt.y;
  lidar_point.point.z = pt.z;

  // Transform LiDAR point to camera frame
  geometry_msgs::msg::PointStamped camera_point;
  tf2::doTransform(lidar_point, camera_point, transform);

  // ignore points behind the camera
  if (camera_point.point.z < 1) {
    return std::nullopt;
  }

  // Convert the projection matrix (std::array) to Eigen::Matrix<double, 3, 4>
  Eigen::Matrix<double, 3, 4> projection_matrix;
  projection_matrix << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11];

  // Convert camera point to Eigen vector (homogeneous coordinates)
  Eigen::Vector4d camera_point_eigen(camera_point.point.x, camera_point.point.y,
                                     camera_point.point.z, 1.0);

  // Project the point onto the image plane using the projection matrix
  Eigen::Vector3d projected_point = projection_matrix * camera_point_eigen;

  // Normalize the projected coordinates
  cv::Point2d proj_pt;
  proj_pt.x = projected_point.x() / projected_point.z();
  proj_pt.y = projected_point.y() / projected_point.z();

  // within the image bounds
  if (proj_pt.x >= 0 && proj_pt.x < image_width_ && proj_pt.y >= 0 && proj_pt.y < image_height_) {
    return proj_pt;
  }

  return std::nullopt;
}

// CLUSTERING FUNCTIONS
// ------------------------------------------------------------------------------------------------

void ProjectionUtils::euclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                 double clusterTolerance, int minClusterSize,
                                                 int maxClusterSize,
                                                 std::vector<pcl::PointIndices>& cluster_indices) {
  /*
      Segments distinct groups of point clouds based on cluster tolerance (euclidean distance) and
     size constraints

      Purpose: Populates a vector of cluster_indices, tells which indexes the cluster is in the
     original cloud
  */

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

void ProjectionUtils::assignClusterColors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                          const std::vector<pcl::PointIndices>& cluster_indices,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr& clustered_cloud) {
  if (cloud->empty() || cluster_indices.empty()) return;

  clustered_cloud->clear();  // Clear previous data
  clustered_cloud->points.reserve(cloud->size());

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);

  for (const auto& indices : cluster_indices) {
    int r = dis(gen);
    int g = dis(gen);
    int b = dis(gen);
    for (const auto& index : indices.indices) {
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

void ProjectionUtils::mergeClusters(std::vector<pcl::PointIndices>& cluster_indices,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    double mergeTolerance) {
  // Precompute stats and delegate to stats-based version
  auto stats = computeClusterStats(cloud, cluster_indices);
  mergeClusters(cluster_indices, cloud, stats, mergeTolerance);
}

void ProjectionUtils::mergeClusters(std::vector<pcl::PointIndices>& cluster_indices,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    const std::vector<ClusterStats>& stats,
                                    double mergeTolerance) {
  /*
      merges two clusters based on the euclidean distance between their centroids, determined by
     merge tolerance

      Purpose: updates cluster_indices with the merged clusters
  */

  if (cloud->empty() || cluster_indices.empty() || stats.size() != cluster_indices.size()) return;

  std::vector<bool> merged(cluster_indices.size(), false);

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (merged[i]) continue;

    // Use precomputed centroid from stats
    const Eigen::Vector4f& centroid_i = stats[i].centroid;

    for (size_t j = i + 1; j < cluster_indices.size(); ++j) {
      if (merged[j]) continue;  // Skip if already merged

      // Use precomputed centroid from stats
      const Eigen::Vector4f& centroid_j = stats[j].centroid;

      // euclidean distance
      double distance = (centroid_i - centroid_j).norm();

      if (distance < mergeTolerance) {
        // Check if merged cluster would be rectangular before merging
        // This prevents merging car + traffic island (which would be too square)
        std::vector<int> merged_indices = cluster_indices[i].indices;
        merged_indices.insert(merged_indices.end(),
                            cluster_indices[j].indices.begin(),
                            cluster_indices[j].indices.end());
        
        // Quick search-based fit check on hypothetical merged cluster
        SearchResult test_fit = computeSearchBasedFit(*cloud, merged_indices);
        if (test_fit.ok) {
          double aspect_ratio = test_fit.len / std::max(test_fit.wid, 0.1f);
          
          // Only merge if aspect ratio is reasonable (1.2 to 7.0)
          // This prevents merging car + traffic island (which would be too square)
          if (aspect_ratio >= 1.8 && aspect_ratio <= 7.0) {
            cluster_indices[i].indices = std::move(merged_indices);
            merged[j] = true;
          }
          // If aspect ratio is bad, skip merging (clusters remain separate)
        } else {
          // If search-based fit fails, don't merge (clusters remain separate)
        }
      }
    }
  }

  // Remove merged clusters from the list
  std::vector<pcl::PointIndices> filtered_clusters;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (!merged[i]) {
      filtered_clusters.push_back(cluster_indices[i]);
    }
  }
  cluster_indices = filtered_clusters;
}

void ProjectionUtils::filterClusterbyDensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             std::vector<pcl::PointIndices>& cluster_indices,
                                             double densityWeight, double sizeWeight,
                                             double distanceWeight, double scoreThreshold) {
  // Precompute stats and delegate to stats-based version
  auto stats = computeClusterStats(cloud, cluster_indices);
  filterClusterbyDensity(stats, cluster_indices, densityWeight, sizeWeight, distanceWeight, scoreThreshold);
}

void ProjectionUtils::filterClusterbyDensity(const std::vector<ClusterStats>& stats,
                                             std::vector<pcl::PointIndices>& cluster_indices,
                                             double densityWeight, double sizeWeight,
                                             double distanceWeight, double scoreThreshold) {
  /*
      Applies weighted scoring of size, density, and distance to filter out clusters, with values
     normalized using arbitrary expected values

      Purpose: updates cluster_indices with removed clusters that are too big, too far, or too
     sparse
  */

  if (stats.size() != cluster_indices.size() || cluster_indices.empty()) return;

  // Define maximum expected values for normalization
  const double max_density = 700.0;  // Maximum density (points/m³)
  const double max_size =
      12.0;  // Maximum cluster size; the diagonal length of its extents (meters)
  const double max_distance = 60.0;  // Maximum distance of a cluster (meters)

  std::vector<pcl::PointIndices> filtered_clusters;

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    const auto& clusters = cluster_indices[i];
    const auto& s = stats[i];

    if (s.num_points < 10) continue;  // Skip small clusters

    // Use precomputed stats instead of scanning points
    double min_x = static_cast<double>(s.min_x);
    double max_x = static_cast<double>(s.max_x);
    double min_y = static_cast<double>(s.min_y);
    double max_y = static_cast<double>(s.max_y);
    double min_z = static_cast<double>(s.min_z);
    double max_z = static_cast<double>(s.max_z);

    // calculate cluster size (the diagonal length of the extents of the cluster)
    double cluster_size =
        std::sqrt((max_x - min_x) * (max_x - min_x) + (max_y - min_y) * (max_y - min_y) +
                  (max_z - min_z) * (max_z - min_z));

    // calculate cluster density (points per unit volume)
    double cluster_volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
    double density = cluster_volume > 0 ? static_cast<double>(s.num_points) / cluster_volume : 0;

    // calculate average distance from origin using centroid
    double avg_distance = std::sqrt(s.centroid.x() * s.centroid.x() +
                                    s.centroid.y() * s.centroid.y() +
                                    s.centroid.z() * s.centroid.z());

    // normalize the factors
    double normalized_density = density / max_density;
    double normalized_size = cluster_size / max_size;
    double normalized_distance = avg_distance / max_distance;

    //  weighted score
    double score = (normalized_density * densityWeight) + (normalized_size * sizeWeight) +
                   (normalized_distance * distanceWeight);

    if (score < scoreThreshold) {
      filtered_clusters.push_back(clusters);
    }
  }

  // Replace the original cluster indices with the filtered ones
  cluster_indices = filtered_clusters;
}

bool ProjectionUtils::computeClusterCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             const pcl::PointIndices& cluster_indices,
                                             pcl::PointXYZ& centroid) {
  if (cloud->empty() || cluster_indices.indices.empty()) return false;

  // Compute centroid of the cluster
  Eigen::Vector4f centroid_eigen;
  pcl::compute3DCentroid(*cloud, cluster_indices, centroid_eigen);

  // Assign values
  centroid.x = centroid_eigen[0];
  centroid.y = centroid_eigen[1];
  centroid.z = centroid_eigen[2];

  return true;
}


double ProjectionUtils::computeMaxIOU8Corners(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr&  input_cloud,
  const pcl::PointIndices&                    cluster_indices,
  const geometry_msgs::msg::TransformStamped& transform,
  const std::array<double, 12>&               projection_matrix,
  const vision_msgs::msg::Detection2DArray&   detections,
  const float                                 object_detection_confidence)
{
  // Compute stats for this cluster and delegate to stats-based version
  std::vector<pcl::PointIndices> single_cluster = {cluster_indices};
  auto stats = computeClusterStats(input_cloud, single_cluster);
  if (stats.empty()) return 0.0;
  return computeMaxIOU8Corners(stats[0], transform, projection_matrix, detections, object_detection_confidence);
}

double ProjectionUtils::computeMaxIOU8Corners(
  const ClusterStats&                         cluster_stats,
  const geometry_msgs::msg::TransformStamped& transform,
  const std::array<double, 12>&               projection_matrix,
  const vision_msgs::msg::Detection2DArray&   detections,
  const float                                 object_detection_confidence)
{
  // 1) Use precomputed 3D axis-aligned min/max from stats
  float min_x = cluster_stats.min_x;
  float max_x = cluster_stats.max_x;
  float min_y = cluster_stats.min_y;
  float max_y = cluster_stats.max_y;
  float min_z = cluster_stats.min_z;
  float max_z = cluster_stats.max_z;

  if (cluster_stats.num_points == 0) {
    return 0.0;
  }

  // 2) build all eight corners of the AABB
  std::array<pcl::PointXYZ,8> corners = {{
    {min_x, min_y, min_z}, {min_x, min_y, max_z},
    {min_x, max_y, min_z}, {min_x, max_y, max_z},
    {max_x, min_y, min_z}, {max_x, min_y, max_z},
    {max_x, max_y, min_z}, {max_x, max_y, max_z}
  }};

  // 3) project each corner & form a tight 2D rect
  double u0 =  std::numeric_limits<double>::infinity(),
         v0 =  std::numeric_limits<double>::infinity();
  double u1 = -std::numeric_limits<double>::infinity(),
         v1 = -std::numeric_limits<double>::infinity();

  for (auto &C : corners) {
    auto uv = projectLidarToCamera(transform, projection_matrix, C);
    if (!uv) continue;
    u0 = std::min(u0, uv->x);
    v0 = std::min(v0, uv->y);
    u1 = std::max(u1, uv->x);
    v1 = std::max(v1, uv->y);
  }
  if (u1 <= u0 || v1 <= v0) {
    return 0.0;
  }
  cv::Rect cluster_rect(u0, v0, u1 - u0, v1 - v0);

  // 4) compare to each detection bbox, return the best IoU
  double best_iou = 0.0;
  for (auto &det : detections.detections) {
    if (!det.results.empty() &&
        det.results[0].hypothesis.score < object_detection_confidence)
    {
      continue;
    }
    const auto &b = det.bbox;
    cv::Rect det_rect(
      b.center.position.x - b.size_x/2,
      b.center.position.y - b.size_y/2,
      b.size_x, b.size_y
    );
    double inter = (cluster_rect & det_rect).area();
    double uni   = cluster_rect.area() + det_rect.area() - inter;
    if (uni > 0.0) {
      best_iou = std::max(best_iou, inter/uni);
    }
  }
  return best_iou;
}

void ProjectionUtils::computeHighestIOUCluster(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
  std::vector<pcl::PointIndices>& cluster_indices,
  const vision_msgs::msg::Detection2DArray& detections,
  const geometry_msgs::msg::TransformStamped& transform,
  const std::array<double, 12>& projection_matrix,
  const float object_detection_confidence)
{
  // Precompute stats and delegate to stats-based version
  auto stats = computeClusterStats(input_cloud, cluster_indices);
  computeHighestIOUCluster(stats, cluster_indices, detections, transform, projection_matrix, object_detection_confidence);
}

void ProjectionUtils::computeHighestIOUCluster(
  const std::vector<ClusterStats>& stats,
  std::vector<pcl::PointIndices>& cluster_indices,
  const vision_msgs::msg::Detection2DArray& detections,
  const geometry_msgs::msg::TransformStamped& transform,
  const std::array<double, 12>& projection_matrix,
  const float object_detection_confidence)
{
if (stats.size() != cluster_indices.size() || cluster_indices.empty()) {
  return;
}

// Early return if there are no YOLO detections - no point in matching clusters
// This prevents creating bounding boxes without corresponding YOLO detections
if (detections.detections.empty()) {
  cluster_indices.clear();
  return;
}

// Compute IoU for each cluster and find the single best one
double best_overall_iou = 0.0;
size_t best_cluster_idx = 0;
bool found_valid_cluster = false;

for (size_t i = 0; i < cluster_indices.size(); ++i) {
  // computeMaxIOU8Corners projects the 8 AABB corners and returns the best IoU
  // Use precomputed stats instead of scanning points
  double iou = computeMaxIOU8Corners(
    stats[i],
    transform,
    projection_matrix,
    detections,
    object_detection_confidence
  );

  if (iou > best_overall_iou) {
    best_overall_iou = iou;
    best_cluster_idx = i;
    found_valid_cluster = true;
  }
}

// Only keep the single best cluster if it has a reasonable IoU
// Use a minimum threshold to avoid keeping clusters with very low IoU
const double min_iou_threshold = 0.2;  // Minimum IoU to consider a valid match
std::vector<pcl::PointIndices> kept_clusters;

if (found_valid_cluster && best_overall_iou >= min_iou_threshold) {
  kept_clusters.push_back(cluster_indices[best_cluster_idx]);
}

// Replace with only the best cluster (or empty if no good match)
cluster_indices = std::move(kept_clusters);
}

// BOUNDING BOX FUNCTIONS
// --------------------------------------------------------------------------------------------

visualization_msgs::msg::MarkerArray ProjectionUtils::computeBoundingBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const sensor_msgs::msg::PointCloud2& msg) {
  // Precompute boxes to avoid recomputing
  auto boxes = computeClusterBoxes(cloud, cluster_indices);
  return computeBoundingBox(boxes, cluster_indices, msg);
}

visualization_msgs::msg::MarkerArray ProjectionUtils::computeBoundingBox(
    const std::vector<Box3D>& boxes,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const sensor_msgs::msg::PointCloud2& msg) {
  visualization_msgs::msg::MarkerArray marker_array;
  if (boxes.size() != cluster_indices.size()) return marker_array;

  int id = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.empty()) continue;

    const auto& box = boxes[i];

    // Build marker (upright box: yaw about Z only)
    visualization_msgs::msg::Marker bbox_marker;
    bbox_marker.header = msg.header;
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

    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.2f;

    bbox_marker.lifetime = rclcpp::Duration::from_seconds(0.15);

    marker_array.markers.push_back(bbox_marker);
  }

  return marker_array;
}

// compute detection 3d array
vision_msgs::msg::Detection3DArray ProjectionUtils::compute3DDetection(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const sensor_msgs::msg::PointCloud2& msg)
{
  // Precompute boxes to avoid recomputing
  auto boxes = computeClusterBoxes(cloud, cluster_indices);
  return compute3DDetection(boxes, cluster_indices, msg);
}

vision_msgs::msg::Detection3DArray ProjectionUtils::compute3DDetection(
    const std::vector<Box3D>& boxes,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const sensor_msgs::msg::PointCloud2& msg)
{
  vision_msgs::msg::Detection3DArray det_arr;
  // copy the same header you used for the markers
  det_arr.header = msg.header;

  if (boxes.size() != cluster_indices.size()) return det_arr;

  int id = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.empty()) continue;

    const auto& box = boxes[i];

    // fill in a Detection3D
    vision_msgs::msg::Detection3D det;
    det.header = msg.header;

    // Hypothesis
    vision_msgs::msg::ObjectHypothesisWithPose hypo;
    hypo.hypothesis.class_id = "cluster";    // or whatever label
    hypo.hypothesis.score    = 1.0;          // or your confidence metric
    det.results.push_back(hypo);

    // Bounding box center
    geometry_msgs::msg::Pose &pose = det.bbox.center;
    pose.position.x = box.center.x();
    pose.position.y = box.center.y();
    pose.position.z = box.center.z();

    // Orientation — yaw from PCA fit (or AABB fallback)
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, box.yaw);
    pose.orientation = tf2::toMsg(quat);

    // Size
    det.bbox.size.x = box.size.x();
    det.bbox.size.y = box.size.y();
    det.bbox.size.z = box.size.z();

    det_arr.detections.push_back(det);
    ++id;
  }

  return det_arr;
}
