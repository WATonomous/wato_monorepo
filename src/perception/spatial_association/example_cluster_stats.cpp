/**
 * Example: Using GPU Cluster Statistics
 * 
 * This file demonstrates how to use the GPU cluster statistics computation
 * from host-side C++ code.
 */

#include "gpu_cluster_stats.hpp"
#include <cuda_runtime.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

/**
 * Convert HostClusterStats to CPU ClusterStats (Eigen::Vector4f format)
 */
void convertToCpuClusterStats(
    const std::vector<HostClusterStats>& gpu_stats,
    std::vector<ProjectionUtils::ClusterStats>& cpu_stats) {
  
  cpu_stats.clear();
  cpu_stats.reserve(gpu_stats.size());
  
  for (const auto& gpu_stat : gpu_stats) {
    ProjectionUtils::ClusterStats stat;
    
    stat.min_x = gpu_stat.min_x;
    stat.max_x = gpu_stat.max_x;
    stat.min_y = gpu_stat.min_y;
    stat.max_y = gpu_stat.max_y;
    stat.min_z = gpu_stat.min_z;
    stat.max_z = gpu_stat.max_z;
    stat.num_points = gpu_stat.num_points;
    
    // Convert centroid to Eigen::Vector4f (x, y, z, 1.0)
    stat.centroid = Eigen::Vector4f(
        gpu_stat.centroid_x,
        gpu_stat.centroid_y,
        gpu_stat.centroid_z,
        1.0f
    );
    
    cpu_stats.push_back(stat);
  }
}

/**
 * Example: Complete workflow for computing cluster statistics
 */
void exampleComputeClusterStats() {
  // ========== Setup: Assume we have voxel-downsampled points and labels ==========
  int M = 20000;  // Number of points after voxel downsampling
  int num_clusters = 150;  // Number of clusters from clustering step
  
  // Allocate device memory for points and labels (assume already allocated)
  float* d_xyz = nullptr;
  int* d_labels = nullptr;
  
  cudaMalloc(&d_xyz, M * 3 * sizeof(float));
  cudaMalloc(&d_labels, M * sizeof(int));
  
  // ... (copy your data to d_xyz and d_labels) ...
  
  // ========== Step 1: Allocate DeviceClusterStats ==========
  DeviceClusterStats d_stats;
  
  int result = allocateDeviceClusterStats(d_stats, num_clusters);
  if (result != 0) {
    std::cerr << "Failed to allocate device cluster stats" << std::endl;
    return;
  }
  
  std::cout << "Allocated device memory for " << num_clusters << " clusters" << std::endl;
  
  // ========== Step 2: Create CUDA stream (optional, for async execution) ==========
  cudaStream_t stream;
  cudaStreamCreate(&stream);
  
  // ========== Step 3: Run GPU cluster statistics computation ==========
  runGpuClusterStats(
      d_xyz,           // Input points
      d_labels,        // Cluster labels
      M,               // Number of points
      num_clusters,    // Number of clusters
      d_stats,         // Device stats structure (output)
      stream           // CUDA stream
  );
  
  // Check for errors
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    std::cerr << "CUDA error: " << cudaGetErrorString(err) << std::endl;
    freeDeviceClusterStats(d_stats);
    cudaStreamDestroy(stream);
    return;
  }
  
  // ========== Step 4: Copy statistics to host ==========
  std::vector<HostClusterStats> h_stats;
  
  copyClusterStatsToHost(d_stats, num_clusters, h_stats, stream);
  
  // Synchronize stream to ensure all operations complete
  cudaStreamSynchronize(stream);
  
  std::cout << "Computed statistics for " << h_stats.size() << " clusters" << std::endl;
  
  // ========== Step 5: Convert to CPU format (if needed) ==========
  std::vector<ProjectionUtils::ClusterStats> cpu_stats;
  convertToCpuClusterStats(h_stats, cpu_stats);
  
  // ========== Step 6: Use the statistics ==========
  for (size_t i = 0; i < cpu_stats.size(); ++i) {
    const auto& stat = cpu_stats[i];
    
    std::cout << "Cluster " << i << ":" << std::endl;
    std::cout << "  Points: " << stat.num_points << std::endl;
    std::cout << "  Centroid: (" 
              << stat.centroid.x() << ", "
              << stat.centroid.y() << ", "
              << stat.centroid.z() << ")" << std::endl;
    std::cout << "  Bounds: x[" << stat.min_x << ", " << stat.max_x << "], "
              << "y[" << stat.min_y << ", " << stat.max_y << "], "
              << "z[" << stat.min_z << ", " << stat.max_z << "]" << std::endl;
  }
  
  // ========== Step 7: Cleanup ==========
  freeDeviceClusterStats(d_stats);
  cudaStreamDestroy(stream);
  cudaFree(d_xyz);
  cudaFree(d_labels);
  
  std::cout << "Done!" << std::endl;
}

/**
 * Example: Integration with GpuContext (reusing buffers)
 */
void exampleWithGpuContext() {
  // Assume we have a GpuContext with pre-allocated buffers
  // (This is a simplified example - actual GpuContext would have more members)
  
  struct GpuContext {
    DeviceClusterStats d_stats;
    // ... other members ...
  };
  
  GpuContext ctx;
  
  // Allocate stats once (reuse across frames)
  int max_clusters = 1000;
  allocateDeviceClusterStats(ctx.d_stats, max_clusters);
  
  // For each frame:
  for (int frame = 0; frame < 100; ++frame) {
    // ... get d_xyz, d_labels, num_clusters for this frame ...
    
    int M = 20000;
    int num_clusters = 150;
    float* d_xyz = nullptr;  // Your points
    int* d_labels = nullptr;  // Your labels
    
    // Reuse the same DeviceClusterStats structure
    // (Note: if num_clusters > max_clusters, you'd need to reallocate)
    if (num_clusters <= ctx.d_stats.num_clusters) {
      runGpuClusterStats(d_xyz, d_labels, M, num_clusters, ctx.d_stats, nullptr);
      
      std::vector<HostClusterStats> h_stats;
      copyClusterStatsToHost(ctx.d_stats, num_clusters, h_stats, nullptr);
      
      // Use h_stats...
    }
  }
  
  // Cleanup once at the end
  freeDeviceClusterStats(ctx.d_stats);
}

/**
 * Example: Performance measurement
 */
void examplePerformance() {
  int M = 50000;
  int num_clusters = 300;
  
  // Allocate
  float* d_xyz = nullptr;
  int* d_labels = nullptr;
  DeviceClusterStats d_stats;
  
  cudaMalloc(&d_xyz, M * 3 * sizeof(float));
  cudaMalloc(&d_labels, M * sizeof(int));
  allocateDeviceClusterStats(d_stats, num_clusters);
  
  // ... initialize d_xyz and d_labels ...
  
  // Measure time
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  
  cudaEventRecord(start);
  
  runGpuClusterStats(d_xyz, d_labels, M, num_clusters, d_stats, nullptr);
  
  std::vector<HostClusterStats> h_stats;
  copyClusterStatsToHost(d_stats, num_clusters, h_stats, nullptr);
  
  cudaEventRecord(stop);
  cudaEventSynchronize(stop);
  
  float elapsed_ms = 0.0f;
  cudaEventElapsedTime(&elapsed_ms, start, stop);
  
  std::cout << "Computed stats for " << num_clusters << " clusters in " 
            << elapsed_ms << " ms" << std::endl;
  std::cout << "Throughput: " << (M / elapsed_ms) << " points/ms" << std::endl;
  
  // Cleanup
  cudaEventDestroy(start);
  cudaEventDestroy(stop);
  cudaFree(d_xyz);
  cudaFree(d_labels);
  freeDeviceClusterStats(d_stats);
}

int main() {
  std::cout << "GPU Cluster Statistics Examples" << std::endl;
  
  // Uncomment to run examples:
  // exampleComputeClusterStats();
  // exampleWithGpuContext();
  // examplePerformance();
  
  return 0;
}

