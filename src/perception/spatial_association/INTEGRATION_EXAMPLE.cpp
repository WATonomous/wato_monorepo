/**
 * Integration Example: GPU Pipeline into spatial_association.cpp
 * 
 * This file shows the exact code changes needed to integrate the GPU pipeline
 * into your existing spatial_association node.
 * 
 * Copy the relevant sections into your spatial_association.cpp file.
 */

#include "spatial_association.hpp"
#include "gpu_pipeline.hpp"
#include "gpu_pipeline_impl.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ============================================================================
// SECTION 1: Modified nonGroundCloudCallback()
// ============================================================================

void spatial_association::nonGroundCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (debug_logging_) {
    RCLCPP_INFO(this->get_logger(), "Received non-ground cloud with %d points", msg->width * msg->height);
  }
  
  // Store the non-ground cloud message from patchwork
  latest_lidar_msg_ = *msg;
  filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL using working object
  pcl::fromROSMsg(latest_lidar_msg_, *working_cloud_);

  if (working_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty non-ground cloud");
    return;
  }

  // NOTE: Voxel filtering is now done on GPU, so we skip CPU voxel filter
  // OLD CODE (remove):
  //   voxel_filter_.setInputCloud(working_cloud_);
  //   voxel_filter_.filter(*working_downsampled_cloud_);
  //   filtered_point_cloud_ = working_downsampled_cloud_;

  // NEW CODE: Use raw cloud, GPU will handle downsampling
  filtered_point_cloud_ = working_cloud_;

  if (debug_logging_) {
    RCLCPP_INFO(this->get_logger(), "Processed non-ground cloud: %zu points (ready for GPU)", 
                filtered_point_cloud_->size());
  }
}

// ============================================================================
// SECTION 2: Modified performClustering() with GPU Integration
// ============================================================================

std::vector<ProjectionUtils::ClusterStats> spatial_association::performClustering(
    std::vector<pcl::PointIndices>& cluster_indices) {
  
  cluster_indices.clear();
  
  if (!filtered_point_cloud_ || filtered_point_cloud_->empty()) {
    return {};
  }

  int N = static_cast<int>(filtered_point_cloud_->size());

  // ========== STEP 1: Convert PCL cloud to contiguous float array ==========
  // Format: [x0,y0,z0, x1,y1,z1, ...]
  std::vector<float> xyz;
  xyz.reserve(N * 3);
  for (size_t i = 0; i < filtered_point_cloud_->size(); ++i) {
    xyz.push_back(filtered_point_cloud_->points[i].x);
    xyz.push_back(filtered_point_cloud_->points[i].y);
    xyz.push_back(filtered_point_cloud_->points[i].z);
  }

  // ========== STEP 2: Setup GPU parameters ==========
  GPUParams params;
  params.voxel_leaf_size_x = 0.2f;  // Match your current voxel filter setting
  params.voxel_leaf_size_y = 0.2f;
  params.voxel_leaf_size_z = 0.2f;
  params.cluster_tolerance = euclid_cluster_tolerance_;
  params.min_cluster_size = euclid_min_cluster_size_;
  params.max_cluster_size = euclid_max_cluster_size_;

  // ========== STEP 3: Run GPU pipeline ==========
  std::vector<int> labels;  // Output: cluster ID per point (-1 = noise)
  std::vector<GPUClusterStats> gpu_clusters;  // Output: per-cluster stats
  
  if (!runGpuPipeline(xyz.data(), N, params, labels, gpu_clusters)) {
    RCLCPP_ERROR(get_logger(), "GPU pipeline failed, falling back to CPU");
    
    // ========== FALLBACK: Use CPU implementation ==========
    ProjectionUtils::euclideanClusterExtraction(filtered_point_cloud_, 
                                                euclid_cluster_tolerance_,
                                                euclid_min_cluster_size_, 
                                                euclid_max_cluster_size_,
                                                cluster_indices);
    auto cluster_stats = ProjectionUtils::computeClusterStats(filtered_point_cloud_, cluster_indices);
    ProjectionUtils::filterClusterbyDensity(filtered_point_cloud_, cluster_indices, 
                                            density_weight_, size_weight_, 
                                            distance_weight_, score_threshold_);
    ProjectionUtils::mergeClusters(cluster_indices, filtered_point_cloud_, cluster_stats, merge_threshold_);
    cluster_stats = ProjectionUtils::computeClusterStats(filtered_point_cloud_, cluster_indices);
    return cluster_stats;
  }

  // ========== STEP 4: Rebuild pcl::PointIndices from labels ==========
  // labels[i] = cluster ID for point i, or -1 if noise
  
  // Find maximum cluster ID
  int max_cluster_id = -1;
  for (int label : labels) {
    if (label > max_cluster_id) {
      max_cluster_id = label;
    }
  }

  if (max_cluster_id < 0) {
    // No valid clusters found
    if (debug_logging_) {
      RCLCPP_DEBUG(get_logger(), "No valid clusters found after GPU clustering");
    }
    return {};
  }

  // Initialize cluster_indices with empty vectors
  cluster_indices.resize(max_cluster_id + 1);
  for (auto& cluster : cluster_indices) {
    cluster.indices.clear();
  }

  // Populate cluster_indices from labels
  for (size_t i = 0; i < labels.size(); ++i) {
    int cluster_id = labels[i];
    if (cluster_id >= 0) {
      cluster_indices[cluster_id].indices.push_back(static_cast<int>(i));
    }
    // Points with label == -1 are noise and are skipped
  }

  // Remove empty clusters (shouldn't happen, but safety check)
  cluster_indices.erase(
    std::remove_if(cluster_indices.begin(), cluster_indices.end(),
                   [](const pcl::PointIndices& ci) { return ci.indices.empty(); }),
    cluster_indices.end()
  );

  // ========== STEP 5: Convert GPUClusterStats to CPU ClusterStats ==========
  std::vector<ProjectionUtils::ClusterStats> cluster_stats;
  cluster_stats.reserve(gpu_clusters.size());
  
  for (const auto& gpu_stat : gpu_clusters) {
    ProjectionUtils::ClusterStats stat;
    stat.centroid = Eigen::Vector4f(gpu_stat.centroid_x, 
                                    gpu_stat.centroid_y, 
                                    gpu_stat.centroid_z, 
                                    1.0f);
    stat.min_x = gpu_stat.min_x;
    stat.max_x = gpu_stat.max_x;
    stat.min_y = gpu_stat.min_y;
    stat.max_y = gpu_stat.max_y;
    stat.min_z = gpu_stat.min_z;
    stat.max_z = gpu_stat.max_z;
    stat.num_points = gpu_stat.num_points;
    cluster_stats.push_back(stat);
  }

  // ========== STEP 6: Apply filtering and merging (using GPU-computed stats) ==========
  // These functions already accept precomputed stats, so they work with GPU stats
  ProjectionUtils::filterClusterbyDensity(cluster_stats, cluster_indices, 
                                          density_weight_, size_weight_, 
                                          distance_weight_, score_threshold_);
  
  ProjectionUtils::mergeClusters(cluster_indices, filtered_point_cloud_, 
                                 cluster_stats, merge_threshold_);

  // ========== STEP 7: Recompute stats after merging ==========
  // TODO: Could avoid this by having GPU pipeline handle merging, or by recomputing only merged clusters
  // For now, we recompute on CPU (fast since we only recompute merged clusters)
  cluster_stats = ProjectionUtils::computeClusterStats(filtered_point_cloud_, cluster_indices);

  if (debug_logging_) {
    RCLCPP_DEBUG(get_logger(), "GPU clustering complete: %zu clusters found", cluster_indices.size());
  }

  return cluster_stats;
}

// ============================================================================
// SECTION 3: Constructor Modifications (add GPU initialization)
// ============================================================================

spatial_association::spatial_association() : Node("spatial_association") {
  initializeParams();

  // Initialize working PCL objects for performance optimization
  working_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  working_downsampled_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  working_colored_cluster_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  working_centroid_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  
  // OLD CODE (remove):
  //   voxel_filter_.setLeafSize(0.2f, 0.2f, 0.2f);

  // NEW CODE: Initialize GPU pipeline with pre-allocated buffers
  // Estimate: 200K points max, 4K clusters max (adjust based on your typical data)
  int max_points = 200000;
  int max_clusters = 4000;
  if (!initializeGpuPipeline(max_points, max_clusters)) {
    RCLCPP_WARN(this->get_logger(), "GPU pipeline initialization failed, will use CPU fallback");
  } else {
    RCLCPP_INFO(this->get_logger(), "GPU pipeline initialized: max_points=%d, max_clusters=%d", 
                max_points, max_clusters);
  }

  // ... rest of constructor remains the same ...
}

// ============================================================================
// SECTION 4: Destructor Modifications (add GPU cleanup)
// ============================================================================

// Add to your destructor (if you have one, or create one):
spatial_association::~spatial_association() {
  // Cleanup GPU pipeline (frees all device memory)
  cleanupGpuPipeline();
  RCLCPP_DEBUG(this->get_logger(), "GPU pipeline cleaned up");
}

// ============================================================================
// SECTION 5: Header File Modifications (spatial_association.hpp)
// ============================================================================

/*
Add to your spatial_association class header:

// OLD CODE (remove):
//   pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

// NEW CODE (optional, for pinned memory optimization):
private:
  float* gpu_input_pinned_ = nullptr;  // Optional: pinned memory for faster transfers
  size_t gpu_input_capacity_ = 0;
*/

// ============================================================================
// SECTION 6: Helper Function Implementation (add to .cpp file)
// ============================================================================

/**
 * Helper function to rebuild pcl::PointIndices from GPU labels.
 * (Alternative implementation, can be added as a utility function)
 */
void rebuildClusterIndicesFromLabels(
    const std::vector<int>& labels,
    std::vector<pcl::PointIndices>& cluster_indices) {
  
  cluster_indices.clear();
  
  // Find maximum cluster ID
  int max_cluster_id = -1;
  for (int label : labels) {
    if (label > max_cluster_id) {
      max_cluster_id = label;
    }
  }

  if (max_cluster_id < 0) {
    return;  // No valid clusters
  }

  // Initialize cluster_indices
  cluster_indices.resize(max_cluster_id + 1);
  for (auto& cluster : cluster_indices) {
    cluster.indices.clear();
  }

  // Populate from labels
  for (size_t i = 0; i < labels.size(); ++i) {
    int cluster_id = labels[i];
    if (cluster_id >= 0) {
      cluster_indices[cluster_id].indices.push_back(static_cast<int>(i));
    }
  }

  // Remove empty clusters
  cluster_indices.erase(
    std::remove_if(cluster_indices.begin(), cluster_indices.end(),
                   [](const pcl::PointIndices& ci) { return ci.indices.empty(); }),
    cluster_indices.end()
  );
}

/**
 * Helper function to convert GPUClusterStats to CPU ClusterStats.
 */
void convertGpuStatsToCpuStats(
    const std::vector<GPUClusterStats>& gpu_stats,
    std::vector<ProjectionUtils::ClusterStats>& cpu_stats) {
  
  cpu_stats.clear();
  cpu_stats.reserve(gpu_stats.size());
  
  for (const auto& gpu_stat : gpu_stats) {
    ProjectionUtils::ClusterStats stat;
    stat.centroid = Eigen::Vector4f(gpu_stat.centroid_x, 
                                    gpu_stat.centroid_y, 
                                    gpu_stat.centroid_z, 
                                    1.0f);
    stat.min_x = gpu_stat.min_x;
    stat.max_x = gpu_stat.max_x;
    stat.min_y = gpu_stat.min_y;
    stat.max_y = gpu_stat.max_y;
    stat.min_z = gpu_stat.min_z;
    stat.max_z = gpu_stat.max_z;
    stat.num_points = gpu_stat.num_points;
    cpu_stats.push_back(stat);
  }
}

// ============================================================================
// END OF INTEGRATION EXAMPLE
// ============================================================================

