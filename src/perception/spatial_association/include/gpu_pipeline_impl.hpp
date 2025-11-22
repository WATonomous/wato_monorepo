#ifndef GPU_PIPELINE_IMPL_HPP
#define GPU_PIPELINE_IMPL_HPP

/**
 * Implementation details for GPU pipeline data layout and memory management.
 * 
 * This file contains the GpuContext structure and helper functions for
 * managing device memory buffers with reuse across frames.
 */

#include "gpu_pipeline.hpp"
#include <cuda_runtime.h>
#include <vector>
#include <algorithm>

/**
 * GPU Context Structure
 * 
 * Holds all pre-allocated device memory buffers that are reused across frames.
 * This eliminates the need for cudaMalloc/cudaFree on every frame.
 */
struct GpuContext {
  // ========== Input/Output Arrays ==========
  float* d_xyz = nullptr;              // [N*3] Input points: [x0,y0,z0, x1,y1,z1, ...]
  int* d_labels_out = nullptr;         // [N] Final cluster labels (mapped to original N points)
  GPUClusterStats* d_cluster_stats = nullptr;  // [num_clusters] Per-cluster statistics

  // ========== Intermediate Buffers ==========
  float* d_xyz_downsampled = nullptr;  // [M*3] Downsampled points (M <= N)
  int* d_voxel_indices = nullptr;      // [N] Voxel ID per point
  bool* d_valid_mask = nullptr;        // [N] True if point is first in its voxel
  int* d_labels = nullptr;             // [M] Cluster ID per downsampled point (-1 = noise)

  // ========== Spatial Hashing Structures ==========
  int* d_hash_table = nullptr;         // Hash table for neighbor search
  int* d_hash_keys = nullptr;         // Hash keys per point
  int* d_hash_values = nullptr;       // Hash values (point indices)

  // ========== Clustering Structures (Union-Find) ==========
  int* d_parent = nullptr;            // [M] Union-find parent array
  int* d_rank = nullptr;               // [M] Union-find rank array
  int* d_cluster_sizes = nullptr;      // [max_clusters] Size of each cluster

  // ========== Temporary Buffers ==========
  float* d_temp_reduce = nullptr;      // Temporary buffer for reductions
  int* d_temp_indices = nullptr;       // Temporary buffer for sorting/indexing

  // ========== Capacity Tracking ==========
  int capacity_points = 0;             // Max points allocated
  int capacity_downsampled = 0;       // Max downsampled points
  int capacity_clusters = 0;           // Max clusters allocated
  int capacity_hash_size = 0;         // Hash table size

  // ========== CUDA Stream ==========
  cudaStream_t stream = nullptr;        // Stream for async operations

  // ========== State ==========
  bool initialized = false;            // Flag to track if initialized
};

/**
 * Initialize GPU context with pre-allocated buffers.
 * 
 * @param ctx Pointer to GpuContext to initialize
 * @param max_points Maximum expected points per frame
 * @param max_clusters Maximum expected clusters per frame
 * @return true on success, false on error
 */
bool initializeGpuContext(GpuContext* ctx, int max_points, int max_clusters);

/**
 * Cleanup and free all device memory in context.
 * 
 * @param ctx Pointer to GpuContext to cleanup
 */
void cleanupGpuContext(GpuContext* ctx);

/**
 * Ensure context has sufficient capacity for given point/cluster counts.
 * Reallocates if needed.
 * 
 * @param ctx Pointer to GpuContext
 * @param N Number of points in current frame
 * @param max_clusters Maximum clusters expected
 * @return true on success, false on error
 */
bool ensureCapacity(GpuContext* ctx, int N, int max_clusters);

/**
 * Helper function to convert PCL point cloud to contiguous float array.
 * 
 * @param cloud Input PCL point cloud
 * @param xyz Output vector: [x0,y0,z0, x1,y1,z1, ...]
 */
template<typename PointT>
void pclToFloatArray(const pcl::PointCloud<PointT>& cloud, std::vector<float>& xyz) {
  xyz.clear();
  xyz.reserve(cloud.size() * 3);
  for (const auto& pt : cloud.points) {
    xyz.push_back(pt.x);
    xyz.push_back(pt.y);
    xyz.push_back(pt.z);
  }
}

/**
 * Helper function to rebuild pcl::PointIndices from GPU labels.
 * 
 * @param labels Input labels: cluster ID per point (-1 = noise)
 * @param cluster_indices Output: vector of PointIndices, one per cluster
 */
void labelsToPointIndices(const std::vector<int>& labels, 
                          std::vector<pcl::PointIndices>& cluster_indices);

/**
 * Helper function to convert GPUClusterStats to CPU ClusterStats.
 * 
 * @param gpu_stats Input GPU cluster statistics
 * @param cpu_stats Output CPU cluster statistics
 */
void gpuStatsToCpuStats(const std::vector<GPUClusterStats>& gpu_stats,
                        std::vector<ProjectionUtils::ClusterStats>& cpu_stats);

#endif // GPU_PIPELINE_IMPL_HPP

