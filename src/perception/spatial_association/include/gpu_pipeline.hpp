#ifndef GPU_PIPELINE_HPP
#define GPU_PIPELINE_HPP

#include <vector>
#include <cuda_runtime.h>

// Structures
struct GPUParams {
    float voxel_leaf_size_x = 0.2f;
    float voxel_leaf_size_y = 0.2f;
    float voxel_leaf_size_z = 0.2f;
    float cluster_tolerance = 0.5f;
    int min_cluster_size = 20;
    int max_cluster_size = 1000;
};

struct GPUClusterStats {
    float centroid_x, centroid_y, centroid_z;
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
    int num_points;
};

// Core Functions
bool initializeGpuPipeline(int max_points, int max_clusters);
void cleanupGpuPipeline();

bool runGpuPipeline(
    const float* h_xyz,
    int N,
    const GPUParams& params,
    std::vector<int>& h_labels_out,
    std::vector<GPUClusterStats>& clusters_out);

#endif // GPU_PIPELINE_HPP
