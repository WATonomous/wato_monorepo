#ifndef GPU_EUCLIDEAN_CLUSTERING_HPP
#define GPU_EUCLIDEAN_CLUSTERING_HPP

#include <cuda_runtime.h>

/**
 * CUDA Euclidean Clustering
 * 
 * Replaces PCL EuclideanClusterExtraction with GPU-accelerated version.
 * Uses spatial hash grid + union-find for connected components.
 * 
 * Algorithm:
 *   1. Build spatial hash grid (3D grid for neighbor lookup)
 *   2. Find neighbors within cluster_tolerance and union them (union-find)
 *   3. Compress labels to dense [0..num_clusters-1] range
 *   4. Filter clusters by size [min_cluster_size, max_cluster_size]
 *   5. Mark filtered points as label = -1 (noise)
 * 
 * @param d_xyz Device pointer to input points [x0,y0,z0, x1,y1,z1, ...] (M*3 floats)
 * @param M Number of input points
 * @param cluster_tolerance Distance threshold for clustering (meters)
 * @param min_cluster_size Minimum points per cluster
 * @param max_cluster_size Maximum points per cluster
 * @param d_labels Output labels per point (size M, -1 = noise/filtered, else cluster ID)
 * @param d_num_clusters Output number of valid clusters (can be nullptr)
 * @param stream CUDA stream for async execution (can be nullptr)
 * @param d_hash_table Optional pre-allocated hash table (size M*2, or nullptr to allocate)
 * @param d_next_point Optional pre-allocated next point array (size M, or nullptr to allocate)
 * @param d_parent Optional pre-allocated parent array (size M, or nullptr to allocate)
 * @param d_rank Optional pre-allocated rank array (size M, or nullptr to allocate)
 * @param d_cluster_sizes Optional pre-allocated cluster sizes (size M, or nullptr to allocate)
 * @param d_root_to_cluster Optional pre-allocated root mapping (size M, or nullptr to allocate)
 * @param d_cluster_id_map Optional pre-allocated cluster ID map (size M, or nullptr to allocate)
 * @param d_num_clusters_temp Optional pre-allocated temp counter (size 1, or nullptr to allocate)
 * @param d_num_valid_clusters Optional pre-allocated valid counter (size 1, or nullptr to allocate)
 * @return 0 on success, -1 on error
 * 
 * Example usage:
 *   float* d_xyz;  // M*3 floats
 *   int* d_labels; // M ints, pre-allocated
 *   int num_clusters;
 *   int result = runGpuClustering(d_xyz, M, 0.5f, 50, 700, d_labels, &num_clusters, stream);
 *   // d_labels[i] = cluster ID (0..num_clusters-1) or -1 (noise/filtered)
 * 
 * Performance note: Pass pre-allocated buffers to avoid per-call allocation overhead.
 */
int runGpuClustering(
    const float* d_xyz,
    int M,
    float cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size,
    int* d_labels,
    int* d_num_clusters,
    cudaStream_t stream = nullptr,
    // Optional pre-allocated buffers (for performance optimization)
    int* d_hash_table = nullptr,
    int* d_next_point = nullptr,
    int* d_parent = nullptr,
    int* d_rank = nullptr,
    int* d_cluster_sizes = nullptr,
    int* d_root_to_cluster = nullptr,
    int* d_cluster_id_map = nullptr,
    int* d_num_clusters_temp = nullptr,
    int* d_num_valid_clusters = nullptr);

#endif // GPU_EUCLIDEAN_CLUSTERING_HPP

