#include "projection_utils.hpp"
#include "orientation_utils.hpp"
#include "gpu_pipeline_cpp.hpp"  // C++-compatible GPU header (no CUDA includes)
#include <cmath>
#include <limits>
#include <array>
#include <algorithm>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <iostream>

namespace {

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
  if (height > OrientationUtils::kTallObjectHeightThreshold) {
    z_threshold = min_z_all + (height * OrientationUtils::kTallObjectCutBottomFraction);
  } else {
    z_threshold = min_z_all + OrientationUtils::kShortObjectCutBottomMeters;
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

  // 4. Perform Fit using Search-Based Fit
  auto orientation_result = OrientationUtils::computeClusterOrientation(*cloud, orientation_indices);

  if (orientation_result.ok) {
    // Use orientation result directly
    box.center.x() = orientation_result.center_xy.x();
    box.center.y() = orientation_result.center_xy.y();
    box.size.x()   = orientation_result.len;
    box.size.y()   = orientation_result.wid;
    box.yaw        = orientation_result.yaw;

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

void ProjectionUtils::gpuEuclideanClusterExtraction(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double clusterTolerance, 
    int minClusterSize,
    int maxClusterSize,
    std::vector<pcl::PointIndices>& cluster_indices) {

  // 1. Early exit if empty
  if (!cloud || cloud->empty()) {
    cluster_indices.clear();
    return;
  }

  int N = static_cast<int>(cloud->size());

  // 2. Data Marshalling: Convert PCL (Array of Structs) to Raw Float Array
  // We resize upfront to avoid reallocations
  // PCL stores points as structures with padding (usually 16 bytes aligned).
  // We need a packed float array (12 bytes per point) for the GPU.
  std::vector<float> raw_points(N * 3);
  for (int i = 0; i < N; ++i) {
    const auto& pt = cloud->points[i];
    raw_points[i * 3 + 0] = pt.x;
    raw_points[i * 3 + 1] = pt.y;
    raw_points[i * 3 + 2] = pt.z;
  }

  // 3. Setup GPU Parameters
  GPUParams params;
  params.voxel_leaf_size_x = 0.2f; // Should match your desired downsampling
  params.voxel_leaf_size_y = 0.2f;
  params.voxel_leaf_size_z = 0.2f;
  params.cluster_tolerance = static_cast<float>(clusterTolerance);
  params.min_cluster_size = minClusterSize;
  params.max_cluster_size = maxClusterSize;

  // 4. Run the GPU Pipeline
  std::vector<int> labels_out;
  std::vector<GPUClusterStats> gpu_stats; // Ignored here, we just want indices
  
  bool success = runGpuPipeline(
      raw_points.data(), 
      N, 
      params, 
      labels_out, 
      gpu_stats
  );

  // 5. Handle Fallback
  if (!success) {
    std::cerr << "[GPU_UTILS] GPU Pipeline failed. Falling back to CPU PCL." << std::endl;
    // Create a non-const copy for the CPU fallback function
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonconst = 
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    euclideanClusterExtraction(cloud_nonconst, clusterTolerance, minClusterSize, maxClusterSize, cluster_indices);
    return;
  }

  // 6. Data Un-Marshalling: Convert Flat Labels to PCL PointIndices
  // GPU gives us: [0, 0, 1, -1, 1, 0...] (Label per point)
  // PCL wants: Cluster 0: [0, 1, 5], Cluster 1: [2, 4]
  
  // Find max cluster ID to resize vector once
  int max_id = -1;
  for(int label : labels_out) {
    if(label > max_id) max_id = label;
  }

  if (max_id < 0) {
    cluster_indices.clear();
    return;
  }

  // Pre-allocate clusters
  cluster_indices.resize(max_id + 1);
  for(auto& c : cluster_indices) {
    c.header = cloud->header;
    c.indices.reserve(N / (max_id + 1)); // Heuristic reservation
  }

  // Fill indices
  for (int i = 0; i < N; ++i) {
    int label = labels_out[i];
    if (label >= 0) {
      cluster_indices[label].indices.push_back(i);
    }
  }

  // Remove any empty clusters (just in case of gaps in ID generation)
  auto new_end = std::remove_if(cluster_indices.begin(), cluster_indices.end(),
      [](const pcl::PointIndices& c){ return c.indices.empty(); });
  cluster_indices.erase(new_end, cluster_indices.end());
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
        OrientationUtils::SearchResult test_fit = OrientationUtils::computeSearchBasedFit(*cloud, merged_indices);
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

int ProjectionUtils::computeBestClusterIndex(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
  const std::vector<pcl::PointIndices>& cluster_indices,
  const vision_msgs::msg::Detection2DArray& detections,
  const geometry_msgs::msg::TransformStamped& transform,
  const std::array<double, 12>& projection_matrix,
  const float object_detection_confidence)
{
  // Precompute stats and delegate to stats-based version
  auto stats = computeClusterStats(input_cloud, cluster_indices);
  return computeBestClusterIndex(stats, cluster_indices, detections, transform, projection_matrix, object_detection_confidence);
}

int ProjectionUtils::computeBestClusterIndex(
  const std::vector<ClusterStats>& stats,
  const std::vector<pcl::PointIndices>& cluster_indices,
  const vision_msgs::msg::Detection2DArray& detections,
  const geometry_msgs::msg::TransformStamped& transform,
  const std::array<double, 12>& projection_matrix,
  const float object_detection_confidence)
{
  if (stats.size() != cluster_indices.size() || cluster_indices.empty()) {
    return -1;
  }

  // Early return if there are no YOLO detections - no point in matching clusters
  // This prevents creating bounding boxes without corresponding YOLO detections
  if (detections.detections.empty()) {
    return -1;
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

  // Only return the best cluster index if it has a reasonable IoU
  // Use a minimum threshold to avoid keeping clusters with very low IoU
  const double min_iou_threshold = 0.05;  // Be permissive to avoid dropping clusters

  if (found_valid_cluster && best_overall_iou >= min_iou_threshold) {
    return static_cast<int>(best_cluster_idx);
  }

  // Debug visibility: surface the best IoU to help tune thresholds when matches fail
  std::cerr << "[ProjectionUtils] No cluster passed IoU threshold. Best IoU: "
            << best_overall_iou << " (threshold " << min_iou_threshold << ")\n";
  return -1;
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
