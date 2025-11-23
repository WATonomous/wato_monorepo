#include "utils/projection_utils.hpp"
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <limits>
#include <array>
#include <algorithm>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

namespace {
// ============================================================================
// Bounding Box Orientation Parameters
// ============================================================================

// Height threshold to classify objects as "tall" (cars, trucks)
// Tall objects: cut bottom 40% for orientation fitting (avoids ground points)
// Short objects: cut bottom 0.2m for orientation fitting
// Tune: Increase if tall objects get wrong orientation, decrease if short objects are affected
constexpr float kTallObjectHeightThreshold = 1.5f;

// Fraction of height to cut from bottom for tall objects (0.0-1.0)
// Tune: Increase to remove more ground points, decrease to keep more points
constexpr float kTallObjectCutBottomFraction = 0.4f;

// Meters to cut from bottom for short objects
// Tune: Increase if ground points affect orientation, decrease to use more points
constexpr float kShortObjectCutBottomMeters = 0.2f;

// Aspect ratio threshold for front-view detection (nearly square objects)
// If aspect ratio < this, use line-of-sight yaw instead of fitted orientation
// Tune: Increase to use line-of-sight for more objects, decrease for tighter fitting
constexpr double kARFrontViewThreshold = 1.2;

// ============================================================================
// Outlier Rejection Parameters
// ============================================================================

// Minimum cluster size to apply outlier rejection (smaller clusters use all points)
// Tune: Increase to apply rejection to larger clusters, decrease for more aggressive filtering
constexpr size_t kOutlierRejectionPointCount = 30;

// Standard deviation multiplier for outlier clipping (4.5 = clip beyond 4.5σ)
// Tune: Increase to keep more edge points, decrease to remove more outliers
constexpr double kOutlierSigmaMultiplier = 4.5;

// ============================================================================
// Orientation Search Parameters
// ============================================================================

// Minimum points required for orientation fitting
// Tune: Increase for more robust fitting, decrease to fit smaller clusters
constexpr size_t kMinPointsForFit = 3;

// Number of points to sample for orientation search (reduces computation)
// Tune: Increase for more accurate fitting (slower), decrease for faster processing
constexpr size_t kDefaultSamplePointCount = 64;

// Coarse search step size in degrees (0° to 90°)
// Tune: Increase for faster search (less accurate), decrease for finer search
constexpr double kCoarseSearchStepDegrees = 10.0;

// Fine search range around best coarse angle (±degrees)
// Tune: Increase to search wider range, decrease for faster refinement
constexpr double kFineSearchRangeDegrees = 5.0;

// Fine search step size in degrees (refinement step)
// Tune: Increase for faster search, decrease for more precise orientation
constexpr double kFineSearchStepDegrees = 2.0;

// ============================================================================
// Orientation Fallback Parameters
// ============================================================================

// Minimum points needed for orientation computation
// Tune: Increase for more robust orientation, decrease to fit smaller clusters
constexpr size_t kMinOrientationPoints = 3;

// Minimum points for fallback orientation method
// Tune: Increase to require more points before fallback, decrease to use fallback earlier
constexpr size_t kMinOrientationPointsForFallback = 5;

// Minimum cluster size to use fallback orientation
// Tune: Increase to use fallback only for larger clusters, decrease for smaller clusters
constexpr size_t kMinClusterSizeForFallback = 10;

// Fraction of top points to use for fallback orientation (0.0-1.0)
// Tune: Increase to use more points, decrease to focus on top of object
constexpr double kTopFractionForFallback = 0.5;

// Minimum width to compute aspect ratio (prevents division by zero)
// Tune: Increase to ignore very narrow objects, decrease to handle thin objects
constexpr double kMinWidthForAspectRatio = 0.1;

// ============================================================================
// Camera Projection Parameters
// ============================================================================

// Minimum Z distance from camera to project point (meters)
// Points closer than this are considered invalid
// Tune: Increase to filter very close points, decrease to keep more points
constexpr double kMinCameraZDistance = 1.0;

// ============================================================================
// Cluster Merging Parameters
// ============================================================================

// Minimum width to compute aspect ratio (float version, meters)
// Tune: Same as kMinWidthForAspectRatio but for float operations
constexpr float kMinWidthForAspectRatioFloat = 0.1f;

// Minimum aspect ratio to merge clusters (length/width)
// Tune: Decrease to merge more clusters, increase to merge fewer
constexpr double kMinAspectRatioForMerge = 1.8;

// Maximum aspect ratio to merge clusters (length/width)
// Tune: Increase to merge more elongated objects, decrease to merge fewer
constexpr double kMaxAspectRatioForMerge = 7.0;


// ============================================================================
// IoU Matching Parameters
// ============================================================================

// Minimum IoU (Intersection over Union) to match cluster with camera detection
// Tune: Increase to require better matches (fewer false positives), decrease to match more (more false positives)
constexpr double kMinIOUThreshold = 0.15;

// ============================================================================
// Ground Noise Filtering Parameters
// ============================================================================

// Minimum height above ground to be considered valid object (meters)
// Tune: Increase to filter more ground artifacts, decrease to keep lower objects
constexpr float kMinHeightAboveGround = 0.3f;

// Maximum Z coordinate for ground plane (negative = below sensor)
// Clusters below this with low height are filtered as ground
// Tune: Decrease (more negative) to filter more ground points, increase to keep more
constexpr float kMaxGroundPlaneZ = -1.5f;

// Minimum points required for valid object
// Tune: Increase to filter more small clusters, decrease to keep smaller objects
constexpr int kMinPointsForValidObject = 15;

// ============================================================================
// Visualization Parameters
// ============================================================================

// Alpha transparency for bounding box markers (0.0 = transparent, 1.0 = opaque)
// Tune: Increase for more visible boxes, decrease for less obtrusive visualization
constexpr float kMarkerAlpha = 0.2f;

// Lifetime of visualization markers in seconds
// Tune: Increase to keep markers longer, decrease for faster updates
constexpr double kMarkerLifetimeSeconds = 0.15;

// Default detection confidence score (0.0-1.0)
// Tune: Adjust based on your confidence scoring system
constexpr double kDefaultDetectionScore = 1.0;

// ============================================================================
// Physical Object Constraints (based on real-world measurements)
// ============================================================================
struct ObjectConstraints {
  // Car dimensions (meters)
  // Tune: Adjust based on your vehicle types (sedans, SUVs, trucks)
  constexpr static float kCarMinHeight = 1.0f;    // Compact car minimum
  constexpr static float kCarMaxHeight = 3.5f;    // Large truck maximum
  constexpr static float kCarMinLength = 2.5f;    // Small car minimum
  constexpr static float kCarMaxLength = 8.0f;    // Long truck maximum
  constexpr static float kCarMinWidth = 1.4f;     // Narrow car minimum
  constexpr static float kCarMaxWidth = 2.8f;     // Wide truck maximum
  constexpr static float kCarMinVolume = 3.5f;    // Minimum volume (m³)
  
  // Pedestrian/Cyclist dimensions (meters)
  // Tune: Adjust for different pedestrian sizes (children, adults, cyclists)
  constexpr static float kPedestrianMinHeight = 0.8f;   // Child minimum
  constexpr static float kPedestrianMaxHeight = 2.2f;   // Tall adult maximum
  constexpr static float kPedestrianMaxWidth = 1.0f;    // Shoulder width maximum
  
  // General filtering constraints
  // Tune: Adjust based on your LiDAR point density and scene characteristics
  constexpr static float kMinPointDensity = 5.0f;        // Minimum points per m³ (too sparse = invalid)
  constexpr static float kMaxPointDensity = 1000.0f;    // Maximum points per m³ (too dense = invalid)
  constexpr static int kMinPointsSmallObject = 10;      // Minimum points for small objects
  constexpr static int kMinPointsLargeObject = 30;       // Minimum points for large objects
  constexpr static float kMaxAspectRatioXY = 8.0f;       // Maximum length/width ratio
  constexpr static float kMaxAspectRatioZ = 15.0f;       // Maximum horizontal/height ratio (for poles)
};

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

std::vector<Eigen::Vector2f> samplePointsXY(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<int>& indices,
    size_t desired_count = kDefaultSamplePointCount) {
  std::vector<Eigen::Vector2f> pts;
  if (indices.empty()) return pts;

  size_t step = 1;
  if (indices.size() > desired_count) {
    step = indices.size() / desired_count;
    if (step == 0) step = 1;
  }

  pts.reserve((indices.size() + step - 1) / step);
  for (size_t i = 0; i < indices.size(); i += step) {
    const auto& p = cloud.points[indices[i]];
    pts.emplace_back(p.x, p.y);
  }
  return pts;
}

SearchResult computeSearchBasedFit(const pcl::PointCloud<pcl::PointXYZ>& pts,
                                   const std::vector<int>& indices) {
  SearchResult r;
  if (indices.size() < kMinPointsForFit) { r.ok = false; return r; }

  auto search_points = samplePointsXY(pts, indices);
  if (search_points.size() < kMinPointsForFit) { r.ok = false; return r; }

  double min_energy = std::numeric_limits<double>::max();
  double best_theta = 0.0;

  std::vector<std::pair<double, double>> rotated_cache;
  rotated_cache.resize(search_points.size());

  auto calculateEdgeEnergy = [&](double theta) -> double {
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    
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

    double energy = 0.0;
    for (const auto& pr : rotated_cache) {
        double dx = std::min(std::abs(pr.first - min_x), std::abs(pr.first - max_x));
        double dy = std::min(std::abs(pr.second - min_y), std::abs(pr.second - max_y));
        
        double d = std::min(dx, dy);
        
        energy += d; 
    }
    
    return energy / static_cast<double>(search_points.size());
  };

  double coarse_step = kCoarseSearchStepDegrees * M_PI / 180.0;
  for (double theta = 0.0; theta < M_PI_2; theta += coarse_step) {
    double energy = calculateEdgeEnergy(theta);
    if (energy < min_energy) {
      min_energy = energy;
      best_theta = theta;
    }
  }

  double fine_range = kFineSearchRangeDegrees * M_PI / 180.0;
  double fine_step = kFineSearchStepDegrees * M_PI / 180.0;
  
  double start_theta = std::max(0.0, best_theta - fine_range);
  double end_theta   = std::min(M_PI_2, best_theta + fine_range);

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
    const pcl::PointCloud<pcl::PointXYZ>& pts,
    const std::vector<int>& indices,
    double yaw,
    const Eigen::Vector2f& base_center,
    Eigen::Vector2f& out_center,
    Eigen::Vector2f& out_size) {
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

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

  const bool apply_outlier_rejection = (indices.size() < kOutlierRejectionPointCount);
  
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

    const double sigma_mul = kOutlierSigmaMultiplier;

    min_x_rot = std::numeric_limits<double>::infinity();
    max_x_rot = -std::numeric_limits<double>::infinity();
    min_y_rot = std::numeric_limits<double>::infinity();
    max_y_rot = -std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < indices.size(); ++i) {
      double val_x = xs_rot[i];
      double val_y = ys_rot[i];

      if (val_x > mean_xr + sigma_mul * std_xr) val_x = mean_xr + sigma_mul * std_xr;
      if (val_x < mean_xr - sigma_mul * std_xr) val_x = mean_xr - sigma_mul * std_xr;
      if (val_y > mean_yr + sigma_mul * std_yr) val_y = mean_yr + sigma_mul * std_yr;
      if (val_y < mean_yr - sigma_mul * std_yr) val_y = mean_yr - sigma_mul * std_yr;

      min_x_rot = std::min(min_x_rot, val_x);
      max_x_rot = std::max(max_x_rot, val_x);
      min_y_rot = std::min(min_y_rot, val_y);
      max_y_rot = std::max(max_y_rot, val_y);
    }
  } else {
    min_x_rot = *std::min_element(xs_rot.begin(), xs_rot.end());
    max_x_rot = *std::max_element(xs_rot.begin(), xs_rot.end());
    min_y_rot = *std::min_element(ys_rot.begin(), ys_rot.end());
    max_y_rot = *std::max_element(ys_rot.begin(), ys_rot.end());
  }

  double center_x_rot = 0.5 * (min_x_rot + max_x_rot);
  double center_y_rot = 0.5 * (min_y_rot + max_y_rot);

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

  std::vector<int> orientation_indices;
  float z_threshold;
  
  if (height > kTallObjectHeightThreshold) {
    z_threshold = min_z_all + (height * kTallObjectCutBottomFraction);
  } else {
    z_threshold = min_z_all + kShortObjectCutBottomMeters;
  }

  for (const auto& pair : z_sorted_indices) {
    if (pair.first >= z_threshold) orientation_indices.push_back(pair.second);
  }

  if (orientation_indices.size() < kMinOrientationPointsForFallback && 
      cluster.indices.size() > kMinClusterSizeForFallback) {
    std::sort(z_sorted_indices.begin(), z_sorted_indices.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; }); 
    orientation_indices.clear();
    size_t count_to_take = static_cast<size_t>(z_sorted_indices.size() * kTopFractionForFallback); 
    if (count_to_take < kMinOrientationPointsForFallback) count_to_take = z_sorted_indices.size();
    for (size_t i = 0; i < count_to_take; ++i) orientation_indices.push_back(z_sorted_indices[i].second);
  }
  if (orientation_indices.size() < kMinOrientationPoints) orientation_indices = cluster.indices;

  box.center.z() = 0.5f * (min_z_all + max_z_all);
  box.size.z()   = std::max(0.0f, max_z_all - min_z_all);

  SearchResult fit_result = computeSearchBasedFit(*cloud, orientation_indices);

  if (fit_result.ok) {
    double ar = static_cast<double>(fit_result.len) /
                std::max(static_cast<double>(fit_result.wid), kMinWidthForAspectRatio);

    Eigen::Vector4f centroid_temp;
    pcl::compute3DCentroid(*cloud, orientation_indices, centroid_temp);
    double yaw_los = std::atan2(centroid_temp.y(), centroid_temp.x());

    double final_yaw;

    if (ar < kARFrontViewThreshold) {
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
    box.size.x()   = box_size_xy.x();
    box.size.y()   = box_size_xy.y();
    box.yaw        = final_yaw;

  } else {
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

// ============================================================================
// Multi-Stage Filtering System
// ============================================================================

namespace {

class ClusterFilter {
public:
  struct FilterStats {
    int total_input = 0;
    int removed_noise = 0;
    int removed_geometry = 0;
    int removed_density = 0;
    int removed_distance = 0;
    int final_output = 0;
    
    void print() const {
      std::cout << "Cluster Filtering Results:\n"
                << "  Input: " << total_input << "\n"
                << "  Removed (noise): " << removed_noise << "\n"
                << "  Removed (geometry): " << removed_geometry << "\n"
                << "  Removed (density): " << removed_density << "\n"
                << "  Removed (distance): " << removed_distance << "\n"
                << "  Output: " << final_output << "\n";
    }
  };

  // Stage 1: Remove obvious noise
  static void filterNoise(
      const std::vector<ProjectionUtils::ClusterStats>& stats,
      std::vector<pcl::PointIndices>& cluster_indices,
      FilterStats* filter_stats = nullptr) {
    
    std::vector<pcl::PointIndices> kept;
    kept.reserve(cluster_indices.size());
    
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
      const auto& s = stats[i];
      bool keep = true;
      
      // Too few points
      if (s.num_points < 5) {
        keep = false;
      }
      
      // Extremely small objects (likely noise)
      float max_dim = std::max({s.max_x - s.min_x, 
                                s.max_y - s.min_y, 
                                s.max_z - s.min_z});
      if (max_dim < 0.15f) {  // 15cm - smaller than any real object
        keep = false;
      }
      
      if (keep) {
        kept.push_back(cluster_indices[i]);
      } else if (filter_stats) {
        filter_stats->removed_noise++;
      }
    }
    
    cluster_indices = std::move(kept);
  }

  // Stage 2: Geometry-based filtering (physical plausibility)
  static void filterByGeometry(
      const std::vector<ProjectionUtils::ClusterStats>& stats,
      std::vector<pcl::PointIndices>& cluster_indices,
      FilterStats* filter_stats = nullptr) {
    
    std::vector<pcl::PointIndices> kept;
    kept.reserve(cluster_indices.size());
    
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
      const auto& s = stats[i];
      
      float width_x = s.max_x - s.min_x;
      float width_y = s.max_y - s.min_y;
      float height = s.max_z - s.min_z;
      
      // Sort dimensions to get length, width, height
      std::array<float, 3> dims = {width_x, width_y, height};
      std::sort(dims.begin(), dims.end(), std::greater<float>());
      float length = dims[0];  // Longest
      float width = dims[1];   // Middle
      float h = dims[2];       // Shortest
      
      bool keep = true;
      
      // Check 1: Unrealistic sizes
      if (length > 15.0f || width > 4.0f || height > 5.0f) {
        keep = false;  // Larger than any vehicle
      }
      
      // Check 2: Impossible aspect ratios
      if (width > 0.05f) {
        float aspect_ratio = length / width;
        if (aspect_ratio > ObjectConstraints::kMaxAspectRatioXY) {
          keep = false;  // Too elongated (likely wall/fence artifact)
        }
      }
      
      // Check 3: Vertical poles/posts (false positives)
      if (h > 0.1f) {
        float vertical_aspect = std::max(length, width) / h;
        if (vertical_aspect > ObjectConstraints::kMaxAspectRatioZ && 
            std::max(length, width) < 0.5f) {
          keep = false;  // Thin vertical pole
        }
      }
      
      // Check 4: Impossible point counts for size
      float volume = width_x * width_y * height;
      if (volume > 0.01f) {
        float density = s.num_points / volume;
        
        // Too sparse (missing most of object)
        if (density < ObjectConstraints::kMinPointDensity) {
          keep = false;
        }
        
        // Too dense (impossible - would need <1cm point spacing)
        if (density > ObjectConstraints::kMaxPointDensity) {
          keep = false;
        }
      }
      
      if (keep) {
        kept.push_back(cluster_indices[i]);
      } else if (filter_stats) {
        filter_stats->removed_geometry++;
      }
    }
    
    cluster_indices = std::move(kept);
  }

  // Stage 3: Density and quality filtering
  static void filterByQuality(
      const std::vector<ProjectionUtils::ClusterStats>& stats,
      std::vector<pcl::PointIndices>& cluster_indices,
      double max_distance = 60.0,
      FilterStats* filter_stats = nullptr) {
    
    std::vector<pcl::PointIndices> kept;
    kept.reserve(cluster_indices.size());
    
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
      const auto& s = stats[i];
      
      float width_x = s.max_x - s.min_x;
      float width_y = s.max_y - s.min_y;
      float height = s.max_z - s.min_z;
      float volume = width_x * width_y * height;
      
      // Distance from sensor
      double distance = std::sqrt(s.centroid.x() * s.centroid.x() +
                                  s.centroid.y() * s.centroid.y() +
                                  s.centroid.z() * s.centroid.z());
      
      bool keep = true;
      
      // Check 1: Distance-adaptive point threshold
      int min_points = ObjectConstraints::kMinPointsSmallObject;
      if (distance > 30.0) {
        min_points = 8;  // Allow sparser clusters at distance
      } else if (distance > 20.0) {
        min_points = 12;
      } else if (volume > 8.0f) {
        min_points = ObjectConstraints::kMinPointsLargeObject;
      }
      
      if (s.num_points < min_points) {
        keep = false;
      }
      
      // Check 2: Quality score based on completeness
      if (volume > 0.01f) {
        float expected_points = volume * 50.0f;  // Rough estimate
        if (expected_points > 0.0f) {
          float completeness = std::min(1.0f, s.num_points / expected_points);
          
          // At close range, require higher completeness
          float min_completeness = (distance < 15.0) ? 0.15f : 0.08f;
          if (completeness < min_completeness) {
            keep = false;
          }
        }
      }
      
      // Check 3: Maximum distance filtering
      if (distance > max_distance) {
        keep = false;
      }
      
      if (keep) {
        kept.push_back(cluster_indices[i]);
      } else if (filter_stats) {
        if (distance > max_distance) {
          filter_stats->removed_distance++;
        } else {
          filter_stats->removed_density++;
        }
      }
    }
    
    cluster_indices = std::move(kept);
  }
};

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

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, c.indices, centroid);
    s.centroid = centroid;

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
  geometry_msgs::msg::PointStamped lidar_point;
  lidar_point.point.x = pt.x;
  lidar_point.point.y = pt.y;
  lidar_point.point.z = pt.z;

  geometry_msgs::msg::PointStamped camera_point;
  tf2::doTransform(lidar_point, camera_point, transform);

  if (camera_point.point.z < kMinCameraZDistance) {
    return std::nullopt;
  }

  Eigen::Matrix<double, 3, 4> projection_matrix;
  projection_matrix << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11];

  Eigen::Vector4d camera_point_eigen(camera_point.point.x, camera_point.point.y,
                                     camera_point.point.z, 1.0);

  Eigen::Vector3d projected_point = projection_matrix * camera_point_eigen;

  cv::Point2d proj_pt;
  proj_pt.x = projected_point.x() / projected_point.z();
  proj_pt.y = projected_point.y() / projected_point.z();

  if (proj_pt.x >= 0 && proj_pt.x < image_width_ && proj_pt.y >= 0 && proj_pt.y < image_height_) {
    return proj_pt;
  }

  return std::nullopt;
}

void ProjectionUtils::euclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                 double clusterTolerance, int minClusterSize,
                                                 int maxClusterSize,
                                                 std::vector<pcl::PointIndices>& cluster_indices) {
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double base_cluster_tolerance,
    int minClusterSize,
    int maxClusterSize,
    std::vector<pcl::PointIndices>& cluster_indices,
    double close_threshold,
    double close_tolerance_mult) {
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
    const auto& pt = cloud->points[i];
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

  if (!close_cloud->empty()) {
    double close_tolerance = base_cluster_tolerance * close_tolerance_mult;
    std::vector<pcl::PointIndices> close_clusters;
    euclideanClusterExtraction(close_cloud, close_tolerance, minClusterSize, maxClusterSize, close_clusters);
    
    for (auto& cluster : close_clusters) {
      pcl::PointIndices mapped_cluster;
      mapped_cluster.indices.reserve(cluster.indices.size());
      for (int idx : cluster.indices) {
        mapped_cluster.indices.push_back(close_indices_map[idx]);
      }
      cluster_indices.push_back(mapped_cluster);
    }
  }

  if (!far_cloud->empty()) {
    std::vector<pcl::PointIndices> far_clusters;
    euclideanClusterExtraction(far_cloud, base_cluster_tolerance, minClusterSize, maxClusterSize, far_clusters);
    
    for (auto& cluster : far_clusters) {
      pcl::PointIndices mapped_cluster;
      mapped_cluster.indices.reserve(cluster.indices.size());
      for (int idx : cluster.indices) {
        mapped_cluster.indices.push_back(far_indices_map[idx]);
      }
      cluster_indices.push_back(mapped_cluster);
    }
  }
}

void ProjectionUtils::assignClusterColors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                          const std::vector<pcl::PointIndices>& cluster_indices,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr& clustered_cloud) {
  if (cloud->empty() || cluster_indices.empty()) return;

  clustered_cloud->clear();
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
                                    const std::vector<ClusterStats>& stats,
                                    double mergeTolerance) {
  if (cloud->empty() || cluster_indices.empty() || stats.size() != cluster_indices.size()) return;

  std::vector<bool> merged(cluster_indices.size(), false);

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (merged[i]) continue;

    const Eigen::Vector4f& centroid_i = stats[i].centroid;

    for (size_t j = i + 1; j < cluster_indices.size(); ++j) {
      if (merged[j]) continue;

      const Eigen::Vector4f& centroid_j = stats[j].centroid;

      double distance = (centroid_i - centroid_j).norm();

      if (distance < mergeTolerance) {
        std::vector<int> merged_indices = cluster_indices[i].indices;
        merged_indices.insert(merged_indices.end(),
                            cluster_indices[j].indices.begin(),
                            cluster_indices[j].indices.end());
        
        SearchResult test_fit = computeSearchBasedFit(*cloud, merged_indices);
        if (test_fit.ok) {
          double aspect_ratio = test_fit.len / std::max(test_fit.wid, kMinWidthForAspectRatioFloat);
          
          if (aspect_ratio >= kMinAspectRatioForMerge && aspect_ratio <= kMaxAspectRatioForMerge) {
            cluster_indices[i].indices = std::move(merged_indices);
            merged[j] = true;
          }
        }
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

void ProjectionUtils::filterClusterByQuality(
    const std::vector<ClusterStats>& stats,
    std::vector<pcl::PointIndices>& cluster_indices,
    double max_distance,
    bool enable_debug) {
  
  if (stats.size() != cluster_indices.size() || cluster_indices.empty()) {
    return;
  }
  
  ClusterFilter::FilterStats filter_stats;
  filter_stats.total_input = cluster_indices.size();
  
  // Stage 1: Remove obvious noise
  ClusterFilter::filterNoise(stats, cluster_indices, &filter_stats);
  
  // Recompute stats after stage 1
  // (In practice, you might pass the cloud and recompute here)
  
  // Stage 2: Geometry-based filtering
  ClusterFilter::filterByGeometry(stats, cluster_indices, &filter_stats);
  
  // Stage 3: Quality and density filtering
  ClusterFilter::filterByQuality(stats, cluster_indices, max_distance, &filter_stats);
  
  filter_stats.final_output = cluster_indices.size();
  
  if (enable_debug) {
    filter_stats.print();
  }
}

void ProjectionUtils::filterClustersByPhysicsConstraints(
    const std::vector<ClusterStats>& stats,
    std::vector<pcl::PointIndices>& cluster_indices,
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
    float max_aspect_ratio) {
  
  // Hardcoded thresholds (too specific to be configurable)
  constexpr float kMinDimensionForAspect = 0.05f;  // Minimum dimension to check aspect ratio (m)
  constexpr float kVolumeThresholdDensity = 0.01f;  // Minimum volume to check density (m³)
  
  if (stats.size() != cluster_indices.size() || cluster_indices.empty()) {
    return;
  }

  std::vector<pcl::PointIndices> kept_clusters;
  kept_clusters.reserve(cluster_indices.size());

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    const auto& s = stats[i];
    
    // Compute basic dimensions
    float width_x = s.max_x - s.min_x;
    float width_y = s.max_y - s.min_y;
    float height = s.max_z - s.min_z;
    float volume = width_x * width_y * height;
    
    // Distance from sensor
    double distance = std::sqrt(s.centroid.x() * s.centroid.x() +
                                s.centroid.y() * s.centroid.y() +
                                s.centroid.z() * s.centroid.z());
    
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

void ProjectionUtils::filterGroundNoise(
    const std::vector<ClusterStats>& stats,
    std::vector<pcl::PointIndices>& cluster_indices) {
  if (stats.size() != cluster_indices.size() || cluster_indices.empty()) {
    return;
  }

  std::vector<pcl::PointIndices> filtered_clusters;
  filtered_clusters.reserve(cluster_indices.size());

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    const auto& s = stats[i];

    // Filter 1: Skip if num_points < kMinPointsForValidObject
    if (s.num_points < kMinPointsForValidObject) {
      continue;
    }

    float height = s.max_z - s.min_z;

    // Filter 2: Skip if height < kMinHeightAboveGround
    if (height < kMinHeightAboveGround) {
      continue;
    }

    // Filter 3: Skip if cluster center z is < kMaxGroundPlaneZ AND height < 0.5f
    if (s.centroid.z() < kMaxGroundPlaneZ && height < 0.5f) {
      continue;
    }

    float max_horizontal_width = std::max(s.max_x - s.min_x, s.max_y - s.min_y);

    // Filter 4: Skip if height < 0.4f AND max horizontal width > 3.0f
    if (height < 0.4f && max_horizontal_width > 3.0f) {
      continue;
    }

    // Filter 5: Skip if aspect ratio (max_horizontal/height) > 10.0f
    if (height > 0.0f) {
      float aspect_ratio = max_horizontal_width / height;
      if (aspect_ratio > 10.0f) {
        continue;
      }
    }

    filtered_clusters.push_back(cluster_indices[i]);
  }

  cluster_indices = std::move(filtered_clusters);
}

bool ProjectionUtils::computeClusterCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             const pcl::PointIndices& cluster_indices,
                                             pcl::PointXYZ& centroid) {
  if (cloud->empty() || cluster_indices.indices.empty()) return false;

  Eigen::Vector4f centroid_eigen;
  pcl::compute3DCentroid(*cloud, cluster_indices, centroid_eigen);

  centroid.x = centroid_eigen[0];
  centroid.y = centroid_eigen[1];
  centroid.z = centroid_eigen[2];

  return true;
}

double ProjectionUtils::computeMaxIOU8Corners(
  const ClusterStats&                         cluster_stats,
  const geometry_msgs::msg::TransformStamped& transform,
  const std::array<double, 12>&               projection_matrix,
  const vision_msgs::msg::Detection2DArray&   detections,
  const float                                 object_detection_confidence)
{
  float min_x = cluster_stats.min_x;
  float max_x = cluster_stats.max_x;
  float min_y = cluster_stats.min_y;
  float max_y = cluster_stats.max_y;
  float min_z = cluster_stats.min_z;
  float max_z = cluster_stats.max_z;

  if (cluster_stats.num_points == 0) {
    return 0.0;
  }

  std::array<pcl::PointXYZ,8> corners = {{
    {min_x, min_y, min_z}, {min_x, min_y, max_z},
    {min_x, max_y, min_z}, {min_x, max_y, max_z},
    {max_x, min_y, min_z}, {max_x, min_y, max_z},
    {max_x, max_y, min_z}, {max_x, max_y, max_z}
  }};

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

if (detections.detections.empty()) {
  cluster_indices.clear();
  return;
}

  std::vector<std::pair<size_t, double>> cluster_best_ious;
  cluster_best_ious.reserve(cluster_indices.size());

for (size_t i = 0; i < cluster_indices.size(); ++i) {
  double iou = computeMaxIOU8Corners(
    stats[i],
    transform,
    projection_matrix,
    detections,
    object_detection_confidence
  );

    cluster_best_ious.push_back({i, iou});
}

std::vector<pcl::PointIndices> kept_clusters;

  for (const auto& pair : cluster_best_ious) {
    if (pair.second >= kMinIOUThreshold) {
      kept_clusters.push_back(cluster_indices[pair.first]);
    }
}

cluster_indices = std::move(kept_clusters);
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
    bbox_marker.color.a = kMarkerAlpha;

    bbox_marker.lifetime = rclcpp::Duration::from_seconds(kMarkerLifetimeSeconds);

    marker_array.markers.push_back(bbox_marker);
  }

  return marker_array;
}

vision_msgs::msg::Detection3DArray ProjectionUtils::compute3DDetection(
    const std::vector<Box3D>& boxes,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const sensor_msgs::msg::PointCloud2& msg)
{
  vision_msgs::msg::Detection3DArray det_arr;
  det_arr.header = msg.header;

  if (boxes.size() != cluster_indices.size()) return det_arr;

  int id = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.empty()) continue;

    const auto& box = boxes[i];

    vision_msgs::msg::Detection3D det;
    det.header = msg.header;

    vision_msgs::msg::ObjectHypothesisWithPose hypo;
    hypo.hypothesis.class_id = "cluster";
    hypo.hypothesis.score    = kDefaultDetectionScore;
    det.results.push_back(hypo);

    geometry_msgs::msg::Pose &pose = det.bbox.center;
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
    ++id;
  }

  return det_arr;
}
