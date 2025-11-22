#ifndef GPU_CLUSTER_STATS_HPP
#define GPU_CLUSTER_STATS_HPP

#include <cuda_runtime.h>
#include <vector>

/**
 * Device-side cluster statistics structure.
 * Holds device pointers to per-cluster min/max/sum/count arrays.
 */
struct DeviceClusterStats {
  float* d_min_x = nullptr;
  float* d_max_x = nullptr;
  float* d_min_y = nullptr;
  float* d_max_y = nullptr;
  float* d_min_z = nullptr;
  float* d_max_z = nullptr;
  float* d_sum_x = nullptr;
  float* d_sum_y = nullptr;
  float* d_sum_z = nullptr;
  int* d_count = nullptr;
  
  int num_clusters = 0;
  bool initialized = false;
};

/**
 * Host-side cluster statistics (matches CPU ClusterStats).
 * Can be converted to Eigen::Vector4f for centroid.
 */
struct HostClusterStats {
  float min_x, max_x;
  float min_y, max_y;
  float min_z, max_z;
  float centroid_x, centroid_y, centroid_z;
  int num_points;
};

/**
 * Allocate device memory for cluster statistics.
 * 
 * @param d_stats Device cluster statistics structure (output)
 * @param num_clusters Number of clusters
 * @return 0 on success, -1 on error
 */
int allocateDeviceClusterStats(DeviceClusterStats& d_stats, int num_clusters);

/**
 * Free device memory for cluster statistics.
 * 
 * @param d_stats Device cluster statistics structure
 */
void freeDeviceClusterStats(DeviceClusterStats& d_stats);

/**
 * Run GPU cluster statistics computation.
 * 
 * Computes per-cluster min/max/centroid/count using atomic reductions.
 * 
 * @param d_xyz Device pointer to input points [x0,y0,z0, x1,y1,z1, ...] (M*3 floats)
 * @param d_labels Device pointer to cluster labels (size M, -1 = noise)
 * @param M Number of points
 * @param num_clusters Number of clusters
 * @param d_stats Device cluster statistics structure (must be allocated)
 * @param stream CUDA stream for async execution (can be nullptr)
 */
void runGpuClusterStats(
    const float* d_xyz,
    const int* d_labels,
    int M,
    int num_clusters,
    DeviceClusterStats& d_stats,
    cudaStream_t stream = nullptr);

/**
 * Copy cluster statistics from device to host.
 * 
 * @param d_stats Device cluster statistics structure
 * @param num_clusters Number of clusters
 * @param h_stats Output host vector of cluster statistics
 * @param stream CUDA stream for async execution (can be nullptr)
 */
void copyClusterStatsToHost(
    const DeviceClusterStats& d_stats,
    int num_clusters,
    std::vector<HostClusterStats>& h_stats,
    cudaStream_t stream = nullptr);

#endif // GPU_CLUSTER_STATS_HPP

