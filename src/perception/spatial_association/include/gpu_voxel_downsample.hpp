#ifndef GPU_VOXEL_DOWNSAMPLE_HPP
#define GPU_VOXEL_DOWNSAMPLE_HPP

#include <cuda_runtime.h>
#include <cstdint>

/**
 * Run voxel grid downsampling on GPU.
 * 
 * @param d_xyz_in       Device pointer to input points (N * 3 floats)
 * @param N              Number of input points
 * @param voxel_leaf_size Voxel size in meters
 * @param d_xyz_out      Device pointer to output points (max size N * 3 floats)
 * @param stream         CUDA stream
 * @param d_temp_keys    Device pointer for temp keys (size N * sizeof(uint64_t))
 * @param d_temp_vals    Device pointer for temp values (size N * sizeof(int))
 * @return               Number of output points (M), or -1 on error
 */
int runVoxelDownsample(
    float* d_xyz_in,
    int N,
    float voxel_leaf_size,
    float* d_xyz_out,
    cudaStream_t stream,
    uint64_t* d_temp_keys,
    int* d_temp_vals);

#endif // GPU_VOXEL_DOWNSAMPLE_HPP
