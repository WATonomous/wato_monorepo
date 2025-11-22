#ifndef GPU_PIPELINE_CPP_HPP
#define GPU_PIPELINE_CPP_HPP

/**
 * C++-compatible GPU Pipeline Interface
 * 
 * This header provides GPU pipeline types and functions without requiring CUDA headers.
 * Use this in regular C++ files (.cpp) that need to call GPU functions.
 * 
 * For CUDA files (.cu), use gpu_pipeline.hpp instead.
 */

#include <vector>

/**
 * GPU Cluster Statistics (matches CPU ClusterStats but uses plain types)
 */
struct GPUClusterStats {
  float centroid_x, centroid_y, centroid_z;  // x,y,z (w=1.0 implied)
  float min_x, max_x;
  float min_y, max_y;
  float min_z, max_z;
  int num_points;
};

/**
 * GPU Pipeline Parameters
 */
struct GPUParams {
  // Voxel Grid Downsampling
  float voxel_leaf_size_x = 0.2f;
  float voxel_leaf_size_y = 0.2f;
  float voxel_leaf_size_z = 0.2f;
  
  // Euclidean Clustering
  float cluster_tolerance = 0.5f;
  int min_cluster_size = 50;
  int max_cluster_size = 700;
};

/**
 * Main GPU Pipeline Function
 * 
 * Processes a point cloud through voxel downsampling, Euclidean clustering,
 * and cluster statistics computation on the GPU.
 * 
 * @param h_xyz Host input array: [x0,y0,z0, x1,y1,z1, ...] (N points, 3*N floats)
 * @param N Number of points in the input cloud
 * @param params GPU pipeline parameters
 * @param h_labels_out Output: cluster ID per point (-1 = noise/unclustered)
 * @param clusters_out Output: per-cluster statistics
 * @return true on success, false on error
 */
bool runGpuPipeline(
    const float* h_xyz,
    int N,
    const GPUParams& params,
    std::vector<int>& h_labels_out,
    std::vector<GPUClusterStats>& clusters_out);

/**
 * Initialize GPU pipeline (optional, for pre-allocation)
 * 
 * @param max_points Maximum expected points per cloud
 * @param max_clusters Maximum expected clusters per cloud
 * @return true on success, false on error
 */
bool initializeGpuPipeline(int max_points = 0, int max_clusters = 0);

/**
 * Cleanup GPU pipeline (optional)
 * 
 * Frees pre-allocated device memory buffers.
 */
void cleanupGpuPipeline();

#endif // GPU_PIPELINE_CPP_HPP

