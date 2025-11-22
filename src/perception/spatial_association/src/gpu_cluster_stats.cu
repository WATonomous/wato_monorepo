/**
 * CUDA Cluster Statistics Computation
 * 
 * Computes per-cluster statistics (min/max/centroid/count) using atomic reductions.
 */

#include "gpu_cluster_stats.hpp"
#include <cuda_runtime.h>
#include <cstdio>
#include <cmath>
#include <limits>
#include <vector>

// ============================================================================
// Device Cluster Statistics Structure
// ============================================================================
// Note: DeviceClusterStats and HostClusterStats are already defined in 
// gpu_cluster_stats.hpp. Do not redeclare them here!

// ============================================================================
// Atomic Float Min/Max Helpers (CAS Loop Implementation)
// ============================================================================

/**
 * Atomic minimum for floats using Compare-And-Swap loop.
 * This correctly handles negative numbers unlike integer-cast atomicMin.
 * 
 * @param addr Pointer to float value to update
 * @param val New value to compare
 */
__device__ __forceinline__ void atomicMinFloat(float* addr, float val) {
  int* addr_as_int = (int*)addr;
  int old = *addr_as_int;
  int assumed;
  do {
    assumed = old;
    float old_val = __int_as_float(assumed);
    // If val is NOT smaller, stop trying
    if (val >= old_val) break;
    // Try to swap
    old = atomicCAS(addr_as_int, assumed, __float_as_int(val));
  } while (assumed != old);
}

/**
 * Atomic maximum for floats using Compare-And-Swap loop.
 * This correctly handles negative numbers unlike integer-cast atomicMax.
 * 
 * @param addr Pointer to float value to update
 * @param val New value to compare
 */
__device__ __forceinline__ void atomicMaxFloat(float* addr, float val) {
  int* addr_as_int = (int*)addr;
  int old = *addr_as_int;
  int assumed;
  do {
    assumed = old;
    float old_val = __int_as_float(assumed);
    // If val is NOT larger, stop trying
    if (val <= old_val) break;
    // Try to swap
    old = atomicCAS(addr_as_int, assumed, __float_as_int(val));
  } while (assumed != old);
}

// ============================================================================
// Kernel 1: Initialize Statistics Arrays
// ============================================================================

/**
 * Initialize per-cluster statistics arrays.
 * Sets min to +INF, max to -INF, sum to 0, count to 0.
 * 
 * @param d_stats Device cluster statistics structure
 * @param num_clusters Number of clusters
 */
__global__ void initializeClusterStatsKernel(
    DeviceClusterStats d_stats,
    int num_clusters) {
  
  int cluster_id = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (cluster_id >= num_clusters) return;
  
  // Initialize min/max using hex constants for infinity (CUDA-safe)
  // +Infinity: 0x7f800000, -Infinity: 0xff800000
  float pos_inf = __int_as_float(0x7f800000);
  float neg_inf = __int_as_float(0xff800000);
  
  d_stats.d_min_x[cluster_id] = pos_inf;
  d_stats.d_max_x[cluster_id] = neg_inf;
  d_stats.d_min_y[cluster_id] = pos_inf;
  d_stats.d_max_y[cluster_id] = neg_inf;
  d_stats.d_min_z[cluster_id] = pos_inf;
  d_stats.d_max_z[cluster_id] = neg_inf;
  
  // Initialize sums and count
  d_stats.d_sum_x[cluster_id] = 0.0f;
  d_stats.d_sum_y[cluster_id] = 0.0f;
  d_stats.d_sum_z[cluster_id] = 0.0f;
  d_stats.d_count[cluster_id] = 0;
}

// ============================================================================
// Kernel 2: Compute Statistics (Atomic Reductions)
// ============================================================================

/**
 * For each point, atomically update min/max/sum/count for its cluster.
 * 
 * @param d_xyz Input points [x0,y0,z0, x1,y1,z1, ...] (M*3 floats)
 * @param d_labels Cluster labels per point (size M, -1 = noise)
 * @param M Number of points
 * @param d_stats Device cluster statistics structure
 */
__global__ void computeClusterStatsKernel(
    const float* d_xyz,
    const int* d_labels,
    int M,
    DeviceClusterStats d_stats) {
  
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (point_idx >= M) return;
  
  // Get point coordinates
  float x = d_xyz[point_idx * 3 + 0];
  float y = d_xyz[point_idx * 3 + 1];
  float z = d_xyz[point_idx * 3 + 2];
  
  // Get cluster label
  int cluster_id = d_labels[point_idx];
  
  // Skip noise points (label == -1)
  if (cluster_id < 0) return;
  
  // Atomically update min/max/sum/count for this cluster
  // Use CAS loop helpers for correct float min/max with negative numbers
  
  // Atomic min/max for x
  atomicMinFloat(&d_stats.d_min_x[cluster_id], x);
  atomicMaxFloat(&d_stats.d_max_x[cluster_id], x);
  
  // Atomic min/max for y
  atomicMinFloat(&d_stats.d_min_y[cluster_id], y);
  atomicMaxFloat(&d_stats.d_max_y[cluster_id], y);
  
  // Atomic min/max for z
  atomicMinFloat(&d_stats.d_min_z[cluster_id], z);
  atomicMaxFloat(&d_stats.d_max_z[cluster_id], z);
  
  // Atomic add for sums
  atomicAdd(&d_stats.d_sum_x[cluster_id], x);
  atomicAdd(&d_stats.d_sum_y[cluster_id], y);
  atomicAdd(&d_stats.d_sum_z[cluster_id], z);
  
  // Atomic increment for count
  atomicAdd(&d_stats.d_count[cluster_id], 1);
}

// ============================================================================
// Host Helper Functions
// ============================================================================

/**
 * Allocate device memory for cluster statistics.
 * 
 * @param d_stats Device cluster statistics structure (output)
 * @param num_clusters Number of clusters
 * @return 0 on success, -1 on error
 */
int allocateDeviceClusterStats(DeviceClusterStats& d_stats, int num_clusters) {
  if (num_clusters <= 0) {
    return -1;
  }
  
  cudaError_t err;
  
  // Allocate min/max arrays
  err = cudaMalloc(&d_stats.d_min_x, num_clusters * sizeof(float));
  if (err != cudaSuccess) goto cleanup;
  
  err = cudaMalloc(&d_stats.d_max_x, num_clusters * sizeof(float));
  if (err != cudaSuccess) goto cleanup;
  
  err = cudaMalloc(&d_stats.d_min_y, num_clusters * sizeof(float));
  if (err != cudaSuccess) goto cleanup;
  
  err = cudaMalloc(&d_stats.d_max_y, num_clusters * sizeof(float));
  if (err != cudaSuccess) goto cleanup;
  
  err = cudaMalloc(&d_stats.d_min_z, num_clusters * sizeof(float));
  if (err != cudaSuccess) goto cleanup;
  
  err = cudaMalloc(&d_stats.d_max_z, num_clusters * sizeof(float));
  if (err != cudaSuccess) goto cleanup;
  
  // Allocate sum arrays
  err = cudaMalloc(&d_stats.d_sum_x, num_clusters * sizeof(float));
  if (err != cudaSuccess) goto cleanup;
  
  err = cudaMalloc(&d_stats.d_sum_y, num_clusters * sizeof(float));
  if (err != cudaSuccess) goto cleanup;
  
  err = cudaMalloc(&d_stats.d_sum_z, num_clusters * sizeof(float));
  if (err != cudaSuccess) goto cleanup;
  
  // Allocate count array
  err = cudaMalloc(&d_stats.d_count, num_clusters * sizeof(int));
  if (err != cudaSuccess) goto cleanup;
  
  d_stats.num_clusters = num_clusters;
  d_stats.initialized = true;
  
  return 0;

cleanup:
  freeDeviceClusterStats(d_stats);
  return -1;
}

/**
 * Free device memory for cluster statistics.
 * 
 * @param d_stats Device cluster statistics structure
 */
void freeDeviceClusterStats(DeviceClusterStats& d_stats) {
  cudaFree(d_stats.d_min_x);
  cudaFree(d_stats.d_max_x);
  cudaFree(d_stats.d_min_y);
  cudaFree(d_stats.d_max_y);
  cudaFree(d_stats.d_min_z);
  cudaFree(d_stats.d_max_z);
  cudaFree(d_stats.d_sum_x);
  cudaFree(d_stats.d_sum_y);
  cudaFree(d_stats.d_sum_z);
  cudaFree(d_stats.d_count);
  
  d_stats.d_min_x = nullptr;
  d_stats.d_max_x = nullptr;
  d_stats.d_min_y = nullptr;
  d_stats.d_max_y = nullptr;
  d_stats.d_min_z = nullptr;
  d_stats.d_max_z = nullptr;
  d_stats.d_sum_x = nullptr;
  d_stats.d_sum_y = nullptr;
  d_stats.d_sum_z = nullptr;
  d_stats.d_count = nullptr;
  d_stats.num_clusters = 0;
  d_stats.initialized = false;
}

// ============================================================================
// Main Host Function
// ============================================================================

/**
 * Run GPU cluster statistics computation.
 * 
 * @param d_xyz Device pointer to input points [x0,y0,z0, x1,y1,z1, ...] (M*3 floats)
 * @param d_labels Device pointer to cluster labels (size M, -1 = noise)
 * @param M Number of points
 * @param num_clusters Number of clusters
 * @param d_stats Device cluster statistics structure (must be allocated)
 * @param stream CUDA stream for async execution
 * @return 0 on success, -1 on error
 */
void runGpuClusterStats(
    const float* d_xyz,
    const int* d_labels,
    int M,
    int num_clusters,
    DeviceClusterStats& d_stats,
    cudaStream_t stream) {
  
  if (!d_xyz || !d_labels || M <= 0 || num_clusters <= 0 || !d_stats.initialized) {
    return;
  }
  
  // Use default stream if nullptr
  cudaStream_t exec_stream = (stream != nullptr) ? stream : 0;
  
  int threads_per_block = 256;
  
  // ========== Kernel 1: Initialize Statistics ==========
  int num_blocks_clusters = (num_clusters + threads_per_block - 1) / threads_per_block;
  
  initializeClusterStatsKernel<<<num_blocks_clusters, threads_per_block, 0, exec_stream>>>(
      d_stats, num_clusters);
  
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    fprintf(stderr, "Kernel 1 (initialize) error: %s\n", cudaGetErrorString(err));
    return;
  }
  
  // ========== Kernel 2: Compute Statistics ==========
  int num_blocks_points = (M + threads_per_block - 1) / threads_per_block;
  
  computeClusterStatsKernel<<<num_blocks_points, threads_per_block, 0, exec_stream>>>(
      d_xyz, d_labels, M, d_stats);
  
  err = cudaGetLastError();
  if (err != cudaSuccess) {
    fprintf(stderr, "Kernel 2 (compute stats) error: %s\n", cudaGetErrorString(err));
    return;
  }
  
  // Note: Centroids are computed on-the-fly when copying to host
  // The DeviceClusterStats structure now contains all necessary data
}

// ============================================================================
// Copy Statistics to Host
// ============================================================================

/**
 * Copy cluster statistics from device to host and compute centroids.
 * 
 * @param d_stats Device cluster statistics structure
 * @param num_clusters Number of clusters
 * @param h_stats Output host vector of cluster statistics
 * @param stream CUDA stream for async execution
 */
void copyClusterStatsToHost(
    const DeviceClusterStats& d_stats,
    int num_clusters,
    std::vector<HostClusterStats>& h_stats,
    cudaStream_t stream) {
  
  if (!d_stats.initialized || num_clusters <= 0) {
    h_stats.clear();
    return;
  }
  
  cudaStream_t exec_stream = (stream != nullptr) ? stream : 0;
  
  // Allocate temporary host arrays
  std::vector<float> h_min_x(num_clusters);
  std::vector<float> h_max_x(num_clusters);
  std::vector<float> h_min_y(num_clusters);
  std::vector<float> h_max_y(num_clusters);
  std::vector<float> h_min_z(num_clusters);
  std::vector<float> h_max_z(num_clusters);
  std::vector<float> h_sum_x(num_clusters);
  std::vector<float> h_sum_y(num_clusters);
  std::vector<float> h_sum_z(num_clusters);
  std::vector<int> h_count(num_clusters);
  
  // Copy from device to host
  cudaMemcpyAsync(h_min_x.data(), d_stats.d_min_x, num_clusters * sizeof(float),
                  cudaMemcpyDeviceToHost, exec_stream);
  cudaMemcpyAsync(h_max_x.data(), d_stats.d_max_x, num_clusters * sizeof(float),
                  cudaMemcpyDeviceToHost, exec_stream);
  cudaMemcpyAsync(h_min_y.data(), d_stats.d_min_y, num_clusters * sizeof(float),
                  cudaMemcpyDeviceToHost, exec_stream);
  cudaMemcpyAsync(h_max_y.data(), d_stats.d_max_y, num_clusters * sizeof(float),
                  cudaMemcpyDeviceToHost, exec_stream);
  cudaMemcpyAsync(h_min_z.data(), d_stats.d_min_z, num_clusters * sizeof(float),
                  cudaMemcpyDeviceToHost, exec_stream);
  cudaMemcpyAsync(h_max_z.data(), d_stats.d_max_z, num_clusters * sizeof(float),
                  cudaMemcpyDeviceToHost, exec_stream);
  cudaMemcpyAsync(h_sum_x.data(), d_stats.d_sum_x, num_clusters * sizeof(float),
                  cudaMemcpyDeviceToHost, exec_stream);
  cudaMemcpyAsync(h_sum_y.data(), d_stats.d_sum_y, num_clusters * sizeof(float),
                  cudaMemcpyDeviceToHost, exec_stream);
  cudaMemcpyAsync(h_sum_z.data(), d_stats.d_sum_z, num_clusters * sizeof(float),
                  cudaMemcpyDeviceToHost, exec_stream);
  cudaMemcpyAsync(h_count.data(), d_stats.d_count, num_clusters * sizeof(int),
                  cudaMemcpyDeviceToHost, exec_stream);
  
  // Synchronize
  cudaStreamSynchronize(exec_stream);
  
  // Convert to HostClusterStats
  h_stats.clear();
  h_stats.reserve(num_clusters);
  
  for (int i = 0; i < num_clusters; ++i) {
    HostClusterStats stat;
    
    stat.min_x = h_min_x[i];
    stat.max_x = h_max_x[i];
    stat.min_y = h_min_y[i];
    stat.max_y = h_max_y[i];
    stat.min_z = h_min_z[i];
    stat.max_z = h_max_z[i];
    stat.num_points = h_count[i];
    
    // Compute centroid from sum/count
    if (h_count[i] > 0) {
      stat.centroid_x = h_sum_x[i] / static_cast<float>(h_count[i]);
      stat.centroid_y = h_sum_y[i] / static_cast<float>(h_count[i]);
      stat.centroid_z = h_sum_z[i] / static_cast<float>(h_count[i]);
    } else {
      stat.centroid_x = 0.0f;
      stat.centroid_y = 0.0f;
      stat.centroid_z = 0.0f;
    }
    
    // Handle edge case: empty cluster or failed atomic operations
    // Check if min is still at initialized value (+infinity)
    // On host side, we can safely use std::numeric_limits
    if (stat.min_x == std::numeric_limits<float>::infinity()) {
      if (stat.num_points == 0) {
        // Empty cluster - set to zero (will be filtered out downstream)
        stat.min_x = 0.0f;
        stat.max_x = 0.0f;
        stat.min_y = 0.0f;
        stat.max_y = 0.0f;
        stat.min_z = 0.0f;
        stat.max_z = 0.0f;
      } else {
        // This should never happen with correct atomic operations
        // But if it does, it means atomicMin failed - use sum/count as fallback
        fprintf(stderr, "[WARNING] Cluster %d has points (%d) but min_x is still +inf. "
                        "Atomic operations may have failed. Using centroid as fallback.\n", 
                i, stat.num_points);
        // Use centroid ± a small epsilon as bounding box
        float epsilon = 0.1f;
        stat.min_x = stat.centroid_x - epsilon;
        stat.max_x = stat.centroid_x + epsilon;
        stat.min_y = stat.centroid_y - epsilon;
        stat.max_y = stat.centroid_y + epsilon;
        stat.min_z = stat.centroid_z - epsilon;
        stat.max_z = stat.centroid_z + epsilon;
      }
    }
    
    h_stats.push_back(stat);
  }
}

