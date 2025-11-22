#ifndef ORIENTATION_UTILS_HPP
#define ORIENTATION_UTILS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>

namespace OrientationUtils {

// Orientation and filtering thresholds
constexpr float kTallObjectHeightThreshold = 1.5f;
constexpr float kTallObjectCutBottomFraction = 0.4f;
constexpr float kShortObjectCutBottomMeters = 0.2f;
constexpr double kARFrontViewThreshold = 1.4; // front-wall vs elongated

struct SearchResult {
  Eigen::Vector2f center_xy{0.f, 0.f};
  double yaw{0.0};
  float len{0.f};
  float wid{0.f};
  bool ok{false};
};

// Normalize angle to [-π, π]
double normalizeAngle(double a);

// Compute absolute angle difference
double angleDiff(double a, double b);

// Sample points from cloud for orientation search
std::vector<Eigen::Vector2f> samplePointsXY(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<int>& indices,
    size_t desired_count = 64);

// Compute search-based fit for orientation estimation
// Uses edge-alignment fit (minimizes distance of points to bounding box edges)
SearchResult computeSearchBasedFit(const pcl::PointCloud<pcl::PointXYZ>& pts,
                                   const std::vector<int>& indices);

// Compute orientation for a cluster using search-based fit
// Returns the final yaw angle and updates the box center and size
struct OrientationResult {
  Eigen::Vector2f center_xy{0.f, 0.f};
  float len{0.f};
  float wid{0.f};
  double yaw{0.0};
  bool ok{false};
};

OrientationResult computeClusterOrientation(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<int>& orientation_indices);

} // namespace OrientationUtils

#endif // ORIENTATION_UTILS_HPP

