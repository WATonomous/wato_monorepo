/**
 * CUDA Voxel Grid Downsampling
 * Uses Thrust Sort + Unique By Key with pre-allocated buffers.
 */

#include "gpu_voxel_downsample.hpp"
#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/unique.h>
#include <thrust/execution_policy.h>
#include <cstdio>
#include <cstdint>

// ============================================================================
// Constants & Hash
// ============================================================================

static constexpr float X_MIN = -100.0f;
static constexpr float X_MAX = 100.0f;
static constexpr float Y_MIN = -100.0f;
static constexpr float Y_MAX = 100.0f;
static constexpr float Z_MIN = -5.0f;
static constexpr float Z_MAX = 5.0f;
static constexpr int MAX_VOXEL_DIM = 10000;

__device__ inline uint64_t computeVoxelHash(int ix, int iy, int iz) {
  uint64_t key = 0;
  key |= (static_cast<uint64_t>(ix) & 0x1FFFFF) << 43;
  key |= (static_cast<uint64_t>(iy) & 0x1FFFFF) << 22;
  key |= (static_cast<uint64_t>(iz) & 0x3FFFFF);
  return key;
}

// ============================================================================
// Kernel 1: Compute Keys
// ============================================================================

__global__ void computeVoxelKeysKernel(
    const float* d_xyz_in,
    int N,
    float voxel_leaf_size,
    uint64_t* d_keys,
    int* d_vals) {
  
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= N) return;
  
  float x = d_xyz_in[idx * 3 + 0];
  float y = d_xyz_in[idx * 3 + 1];
  float z = d_xyz_in[idx * 3 + 2];
  
  x = fmaxf(X_MIN, fminf(X_MAX, x));
  y = fmaxf(Y_MIN, fminf(Y_MAX, y));
  z = fmaxf(Z_MIN, fminf(Z_MAX, z));
  
  int ix = static_cast<int>((x - X_MIN) / voxel_leaf_size);
  int iy = static_cast<int>((y - Y_MIN) / voxel_leaf_size);
  int iz = static_cast<int>((z - Z_MIN) / voxel_leaf_size);
  
  ix = max(0, min(MAX_VOXEL_DIM - 1, ix));
  iy = max(0, min(MAX_VOXEL_DIM - 1, iy));
  iz = max(0, min(MAX_VOXEL_DIM - 1, iz));
  
  d_keys[idx] = computeVoxelHash(ix, iy, iz);
  d_vals[idx] = idx; 
}

// ============================================================================
// Kernel 2: Gather Points
// ============================================================================

__global__ void gatherPointsKernel(
    const int* d_indices,
    int M,
    const float* d_xyz_in,
    float* d_xyz_out) {
    
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= M) return;

  int point_idx = d_indices[idx];
  
  d_xyz_out[idx * 3 + 0] = d_xyz_in[point_idx * 3 + 0];
  d_xyz_out[idx * 3 + 1] = d_xyz_in[point_idx * 3 + 1];
  d_xyz_out[idx * 3 + 2] = d_xyz_in[point_idx * 3 + 2];
}

// ============================================================================
// Host Wrapper Function
// ============================================================================

int runVoxelDownsample(
    float* d_xyz_in,
    int N,
    float voxel_leaf_size,
    float* d_xyz_out,
    cudaStream_t stream,
    // NEW: Pre-allocated temp buffers passed from pipeline
    uint64_t* d_temp_keys,
    int* d_temp_vals) {
  
  if (!d_xyz_in || !d_xyz_out || !d_temp_keys || !d_temp_vals || N <= 0 || voxel_leaf_size <= 0.0f) {
    return -1;
  }
  
  cudaStream_t exec_stream = (stream != nullptr) ? stream : 0;

  // Wrap raw pointers with Thrust device_ptr
  // This allows us to use Thrust algorithms without allocating new memory
  thrust::device_ptr<uint64_t> t_keys(d_temp_keys);
  thrust::device_ptr<int> t_vals(d_temp_vals);

  // 1. Compute Keys
  int threads_per_block = 256;
  int num_blocks = (N + threads_per_block - 1) / threads_per_block;
  
  computeVoxelKeysKernel<<<num_blocks, threads_per_block, 0, exec_stream>>>(
      d_xyz_in, N, voxel_leaf_size, d_temp_keys, d_temp_vals);
  
  if (cudaGetLastError() != cudaSuccess) return -1;

  // 2. Sort by Key
  thrust::sort_by_key(
      thrust::cuda::par.on(exec_stream),
      t_keys, 
      t_keys + N, 
      t_vals);

  // 3. Unique by Key (Compaction)
  auto new_end = thrust::unique_by_key(
      thrust::cuda::par.on(exec_stream),
      t_keys, 
      t_keys + N, 
      t_vals);

  // Calculate number of unique voxels
  int M = new_end.first - t_keys;

  // 4. Gather Result
  if (M > 0) {
      int gather_blocks = (M + threads_per_block - 1) / threads_per_block;
      gatherPointsKernel<<<gather_blocks, threads_per_block, 0, exec_stream>>>(
          d_temp_vals, // Valid indices are now compacted at the start of this array
          M,
          d_xyz_in,
          d_xyz_out);
  }
  
  return M;
}
