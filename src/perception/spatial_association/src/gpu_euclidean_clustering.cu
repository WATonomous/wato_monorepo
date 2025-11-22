/**
 * CUDA Euclidean Clustering
 * 
 * Replaces PCL EuclideanClusterExtraction with GPU-accelerated version.
 * Uses spatial hash grid + union-find for connected components.
 */

#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/device_ptr.h>
#include <thrust/sequence.h>
#include <thrust/execution_policy.h>
#include <cstdio>
#include <cmath>

// ============================================================================
// Constants
// ============================================================================

static constexpr float X_MIN = -100.0f;
static constexpr float X_MAX = 100.0f;
static constexpr float Y_MIN = -100.0f;
static constexpr float Y_MAX = 100.0f;
static constexpr float Z_MIN = -5.0f;
static constexpr float Z_MAX = 5.0f;

static constexpr int MAX_VOXEL_DIM = 10000;
static constexpr int NOISE_LABEL = -1;

// Hash table parameters
static constexpr int HASH_TABLE_SIZE_MULTIPLIER = 2;  // Hash table = 2x point count
static constexpr int MAX_NEIGHBORS_PER_POINT = 50;    // Max neighbors to check

// ============================================================================
// Spatial Hash Functions
// ============================================================================

/**
 * Compute voxel indices from point coordinates
 */
__device__ inline void pointToVoxel(float x, float y, float z, float cell_size,
                                     int& ix, int& iy, int& iz) {
  // Clamp to bounds
  x = fmaxf(X_MIN, fminf(X_MAX, x));
  y = fmaxf(Y_MIN, fminf(Y_MAX, y));
  z = fmaxf(Z_MIN, fminf(Z_MAX, z));
  
  // Compute voxel indices
  ix = static_cast<int>((x - X_MIN) / cell_size);
  iy = static_cast<int>((y - Y_MIN) / cell_size);
  iz = static_cast<int>((z - Z_MIN) / cell_size);
  
  // Clamp
  ix = max(0, min(MAX_VOXEL_DIM - 1, ix));
  iy = max(0, min(MAX_VOXEL_DIM - 1, iy));
  iz = max(0, min(MAX_VOXEL_DIM - 1, iz));
}

/**
 * Compute hash table index from voxel coordinates
 */
__device__ inline int voxelToHash(int ix, int iy, int iz, int table_size) {
  // Simple hash function
  const int p1 = 73856093;
  const int p2 = 19349663;
  const int p3 = 83492791;
  return ((ix * p1) ^ (iy * p2) ^ (iz * p3)) % table_size;
}

// ============================================================================
// Union-Find Data Structure
// ============================================================================

/**
 * Find root with path compression (Iterative version)
 * Prevents stack overflow and NVLink warnings on GPU
 */
__device__ int findRoot(int* parent, int x) {
  int root = x;
  
  // Pass 1: Find the root
  while (parent[root] != root) {
    root = parent[root];
  }

  // Pass 2: Path compression
  // Point every node on the path directly to the root
  int curr = x;
  while (curr != root) {
    int next = parent[curr];
    parent[curr] = root;
    curr = next;
  }

  return root;
}

/**
 * Union two sets (union by rank)
 */
__device__ void unionSets(int* parent, int* rank, int x, int y) {
  int root_x = findRoot(parent, x);
  int root_y = findRoot(parent, y);
  
  if (root_x == root_y) return;  // Already in same set
  
  // Union by rank
  if (rank[root_x] < rank[root_y]) {
    parent[root_x] = root_y;
  } else if (rank[root_x] > rank[root_y]) {
    parent[root_y] = root_x;
  } else {
    parent[root_y] = root_x;
    rank[root_x]++;
  }
}

// ============================================================================
// Kernel 1: Build Spatial Hash Grid
// ============================================================================

/**
 * Build spatial hash grid for neighbor lookup.
 * Each point is inserted into its voxel cell.
 * 
 * @param d_xyz Input points [x0,y0,z0, x1,y1,z1, ...] (M*3 floats)
 * @param M Number of points
 * @param cell_size Spatial grid cell size (typically = cluster_tolerance)
 * @param d_hash_table Hash table: hash_idx -> first point index in cell
 * @param d_next_point Next point in same cell (linked list)
 * @param table_size Hash table size
 */
__global__ void buildSpatialHashKernel(
    const float* d_xyz,
    int M,
    float cell_size,
    int* d_hash_table,
    int* d_next_point,
    int table_size) {
  
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= M) return;
  
  // Get point coordinates
  float x = d_xyz[idx * 3 + 0];
  float y = d_xyz[idx * 3 + 1];
  float z = d_xyz[idx * 3 + 2];
  
  // Compute voxel indices
  int ix, iy, iz;
  pointToVoxel(x, y, z, cell_size, ix, iy, iz);
  
  // Compute hash index
  int hash_idx = voxelToHash(ix, iy, iz, table_size);
  
  // Insert into hash table (linked list)
  // Use atomic exchange to add to front of list
  int old_head = atomicExch(&d_hash_table[hash_idx], idx);
  d_next_point[idx] = old_head;  // Link to previous head (or -1 if first)
}

// ============================================================================
// Kernel 2: Find Neighbors and Union
// ============================================================================

/**
 * For each point, find neighbors within cluster_tolerance and union them.
 * Checks 3×3×3 neighbor voxel cells.
 * 
 * @param d_xyz Input points
 * @param M Number of points
 * @param cluster_tolerance Distance threshold
 * @param cell_size Spatial grid cell size
 * @param d_hash_table Hash table
 * @param d_next_point Next point in cell (linked list)
 * @param d_parent Union-find parent array
 * @param d_rank Union-find rank array
 * @param table_size Hash table size
 */
__global__ void findNeighborsAndUnionKernel(
    const float* d_xyz,
    int M,
    float cluster_tolerance,
    float cell_size,
    const int* d_hash_table,
    const int* d_next_point,
    int* d_parent,
    int* d_rank,
    int table_size) {
  
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= M) return;
  
  // Get point coordinates
  float x = d_xyz[idx * 3 + 0];
  float y = d_xyz[idx * 3 + 1];
  float z = d_xyz[idx * 3 + 2];
  
  // Compute voxel indices for this point
  int ix, iy, iz;
  pointToVoxel(x, y, z, cell_size, ix, iy, iz);
  
  float tol_sq = cluster_tolerance * cluster_tolerance;
  
  // Check 3×3×3 neighbor voxel cells
  for (int di = -1; di <= 1; ++di) {
    for (int dj = -1; dj <= 1; ++dj) {
      for (int dk = -1; dk <= 1; ++dk) {
        int nix = ix + di;
        int niy = iy + dj;
        int niz = iz + dk;
        
        // Skip if out of bounds
        if (nix < 0 || nix >= MAX_VOXEL_DIM ||
            niy < 0 || niy >= MAX_VOXEL_DIM ||
            niz < 0 || niz >= MAX_VOXEL_DIM) {
          continue;
        }
        
        // Get hash index for neighbor cell
        int hash_idx = voxelToHash(nix, niy, niz, table_size);
        
        // Traverse linked list of points in this cell
        int neighbor_idx = d_hash_table[hash_idx];
        int neighbor_count = 0;
        
        while (neighbor_idx >= 0 && neighbor_count < MAX_NEIGHBORS_PER_POINT) {
          // Skip self
          if (neighbor_idx != idx) {
            // Compute distance
            float dx = x - d_xyz[neighbor_idx * 3 + 0];
            float dy = y - d_xyz[neighbor_idx * 3 + 1];
            float dz = z - d_xyz[neighbor_idx * 3 + 2];
            float dist_sq = dx*dx + dy*dy + dz*dz;
            
            // If within tolerance, union
            if (dist_sq <= tol_sq) {
              unionSets(d_parent, d_rank, idx, neighbor_idx);
            }
          }
          
          // Move to next point in cell
          neighbor_idx = d_next_point[neighbor_idx];
          neighbor_count++;
        }
      }
    }
  }
}

// ============================================================================
// Kernel 3: Compress Labels and Compute Cluster Sizes
// ============================================================================

/**
 * Compress union-find roots to dense labels [0..num_clusters-1]
 * and compute cluster sizes.
 * 
 * @param d_parent Union-find parent array (will be updated with roots)
 * @param M Number of points
 * @param d_labels Output labels (dense [0..num_clusters-1] or -1 for noise)
 * @param d_cluster_sizes Output cluster sizes
 * @param d_num_clusters Output number of clusters
 * @param min_cluster_size Minimum cluster size
 * @param max_cluster_size Maximum cluster size
 */
__global__ void compressLabelsKernel(
    int* d_parent,
    int M,
    int* d_labels,
    int* d_cluster_sizes,
    int* d_num_clusters,
    int min_cluster_size,
    int max_cluster_size) {
  
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  
  // First, find all roots and compress paths
  if (idx < M) {
    d_parent[idx] = findRoot(d_parent, idx);
  }
  
  __syncthreads();
  
  // Thread 0: Initialize cluster counter
  if (idx == 0) {
    *d_num_clusters = 0;
  }
  
  __syncthreads();
  
  // For each point, check if it's a root (parent[idx] == idx)
  if (idx < M) {
    if (d_parent[idx] == idx) {
      // This is a root - assign it a new cluster ID
      int cluster_id = atomicAdd(d_num_clusters, 1);
      d_cluster_sizes[cluster_id] = 0;  // Initialize size
    }
  }
  
  __syncthreads();
  
  // Count cluster sizes (all threads)
  if (idx < M) {
    // Logic was empty/commented out here anyway
  }
}

// ============================================================================
// Kernel 3 (Alternative): Two-Pass Label Compression
// ============================================================================

/**
 * Pass 1: Find all unique roots and assign cluster IDs
 */
__global__ void compressLabelsPass1Kernel(
    const int* d_parent,
    int M,
    int* d_root_to_cluster,  // Map root index -> cluster ID
    int* d_num_clusters) {
  
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (idx == 0) {
    *d_num_clusters = 0;
  }
  
  __syncthreads();
  
  if (idx < M) {
    // Check if this is a root (parent[idx] == idx)
    if (d_parent[idx] == idx) {
      // Assign new cluster ID
      int cluster_id = atomicAdd(d_num_clusters, 1);
      d_root_to_cluster[idx] = cluster_id;
    } else {
      d_root_to_cluster[idx] = -1;  // Not a root
    }
  }
}

/**
 * Pass 2: Map points to cluster IDs
 */
__global__ void compressLabelsPass2Kernel(
    const int* d_parent,
    const int* d_root_to_cluster,
    int M,
    int* d_labels) {
  
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (idx < M) {
    int root = d_parent[idx];
    int cluster_id = d_root_to_cluster[root];
    
    if (cluster_id >= 0) {
      d_labels[idx] = cluster_id;  // Temporary: will filter by size next
    } else {
      d_labels[idx] = NOISE_LABEL;
    }
  }
}

/**
 * Pass 2b: Count cluster sizes
 */
__global__ void countClusterSizesKernel(
    const int* d_labels,
    int M,
    int* d_cluster_sizes,
    int num_clusters) {
  
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (idx < M) {
    int cluster_id = d_labels[idx];
    if (cluster_id >= 0 && cluster_id < num_clusters) {
      atomicAdd(&d_cluster_sizes[cluster_id], 1);
    }
  }
}

/**
 * Pass 3: Filter clusters by size and remap labels
 */
__global__ void filterClustersBySizeKernel(
    const int* d_cluster_sizes,
    int num_clusters,
    int* d_labels,
    int M,
    int min_cluster_size,
    int max_cluster_size,
    int* d_cluster_id_map,  // Map old cluster_id -> new cluster_id (or -1)
    int* d_num_valid_clusters) {
  
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (idx == 0) {
    *d_num_valid_clusters = 0;
  }
  
  __syncthreads();
  
  // Build mapping: valid clusters get new IDs, invalid get -1
  if (idx < num_clusters) {
    int size = d_cluster_sizes[idx];
    if (size >= min_cluster_size && size <= max_cluster_size) {
      int new_id = atomicAdd(d_num_valid_clusters, 1);
      d_cluster_id_map[idx] = new_id;
    } else {
      d_cluster_id_map[idx] = -1;  // Mark as noise
    }
  }
  
  __syncthreads();
  
  // Remap point labels
  if (idx < M) {
    int old_cluster_id = d_labels[idx];
    if (old_cluster_id >= 0) {
      int new_cluster_id = d_cluster_id_map[old_cluster_id];
      d_labels[idx] = new_cluster_id;  // -1 if filtered, else new dense ID
    }
  }
}

// ============================================================================
// Host Wrapper Function
// ============================================================================

/**
 * Run GPU Euclidean clustering.
 * 
 * @param d_xyz Device pointer to input points [x0,y0,z0, x1,y1,z1, ...] (M*3 floats)
 * @param M Number of input points
 * @param cluster_tolerance Distance threshold for clustering
 * @param min_cluster_size Minimum points per cluster
 * @param max_cluster_size Maximum points per cluster
 * @param d_labels Output labels per point (size M, -1 = noise/filtered)
 * @param d_num_clusters Output number of valid clusters
 * @param stream CUDA stream for async execution
 * @return 0 on success, -1 on error
 */
int runGpuClustering(
    const float* d_xyz,
    int M,
    float cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size,
    int* d_labels,
    int* d_num_clusters,
    cudaStream_t stream,
    // Optional pre-allocated buffers (for performance optimization)
    int* d_hash_table,
    int* d_next_point,
    int* d_parent,
    int* d_rank,
    int* d_cluster_sizes,
    int* d_root_to_cluster,
    int* d_cluster_id_map,
    int* d_num_clusters_temp,
    int* d_num_valid_clusters) {
  
  if (!d_xyz || !d_labels || M <= 0 || cluster_tolerance <= 0.0f) {
    return -1;
  }
  
  // Use default stream if nullptr
  cudaStream_t exec_stream = (stream != nullptr) ? stream : 0;
  cudaError_t err;  // Declare once at function start
  
  // Determine if we need to allocate buffers or use pre-allocated ones
  bool use_preallocated = (d_hash_table != nullptr && d_next_point != nullptr && 
                          d_parent != nullptr && d_rank != nullptr &&
                          d_cluster_sizes != nullptr && d_root_to_cluster != nullptr &&
                          d_cluster_id_map != nullptr && d_num_clusters_temp != nullptr &&
                          d_num_valid_clusters != nullptr);
  
  int table_size = M * HASH_TABLE_SIZE_MULTIPLIER;
  
  // Pointers to use (either pre-allocated or from thrust vectors)
  int* d_hash_table_ptr = nullptr;
  int* d_next_point_ptr = nullptr;
  int* d_parent_ptr = nullptr;
  int* d_rank_ptr = nullptr;
  int* d_cluster_sizes_ptr = nullptr;
  int* d_root_to_cluster_ptr = nullptr;
  int* d_cluster_id_map_ptr = nullptr;
  int* d_num_clusters_temp_ptr = nullptr;
  int* d_num_valid_clusters_ptr = nullptr;
  
  // Thrust vectors (only allocated if not using pre-allocated buffers)
  thrust::device_vector<int> d_hash_table_vec;
  thrust::device_vector<int> d_next_point_vec;
  thrust::device_vector<int> d_parent_vec;
  thrust::device_vector<int> d_rank_vec;
  thrust::device_vector<int> d_cluster_sizes_vec;
  thrust::device_vector<int> d_root_to_cluster_vec;
  thrust::device_vector<int> d_cluster_id_map_vec;
  thrust::device_vector<int> d_num_clusters_temp_vec;
  thrust::device_vector<int> d_num_valid_clusters_vec;
  
  if (use_preallocated) {
    // Use pre-allocated buffers (performance optimization)
    d_hash_table_ptr = d_hash_table;
    d_next_point_ptr = d_next_point;
    d_parent_ptr = d_parent;
    d_rank_ptr = d_rank;
    d_cluster_sizes_ptr = d_cluster_sizes;
    d_root_to_cluster_ptr = d_root_to_cluster;
    d_cluster_id_map_ptr = d_cluster_id_map;
    d_num_clusters_temp_ptr = d_num_clusters_temp;
    d_num_valid_clusters_ptr = d_num_valid_clusters;
    
    // Initialize pre-allocated buffers
    cudaMemsetAsync(d_hash_table, -1, table_size * sizeof(int), exec_stream);
    cudaMemsetAsync(d_next_point, -1, M * sizeof(int), exec_stream);
    cudaMemsetAsync(d_rank, 0, M * sizeof(int), exec_stream);
    cudaMemsetAsync(d_cluster_sizes, 0, M * sizeof(int), exec_stream);
    cudaMemsetAsync(d_root_to_cluster, -1, M * sizeof(int), exec_stream);
    cudaMemsetAsync(d_cluster_id_map, -1, M * sizeof(int), exec_stream);
    cudaMemsetAsync(d_num_clusters_temp, 0, sizeof(int), exec_stream);
    cudaMemsetAsync(d_num_valid_clusters, 0, sizeof(int), exec_stream);
    
    // Initialize union-find: each point is its own parent (0, 1, 2, ..., M-1)
    // Use thrust::device_ptr to interface raw CUDA memory with Thrust algorithms
    thrust::device_ptr<int> t_parent(d_parent);
    thrust::sequence(thrust::cuda::par.on(exec_stream), t_parent, t_parent + M);
    
    err = cudaStreamSynchronize(exec_stream);
    if (err != cudaSuccess) {
      fprintf(stderr, "Stream sync error after thrust::sequence: %s\n", cudaGetErrorString(err));
      return -1;
    }
  } else {
    // Allocate temporary device arrays (backward compatibility)
    d_hash_table_vec.resize(table_size, -1);
    d_next_point_vec.resize(M, -1);
    d_parent_vec.resize(M);
    d_rank_vec.resize(M, 0);
    d_cluster_sizes_vec.resize(M, 0);
    d_root_to_cluster_vec.resize(M, -1);
    d_cluster_id_map_vec.resize(M, -1);
    d_num_clusters_temp_vec.resize(1, 0);
    d_num_valid_clusters_vec.resize(1, 0);
    
    // Initialize union-find: each point is its own parent
    thrust::sequence(d_parent_vec.begin(), d_parent_vec.end());
    err = cudaStreamSynchronize(exec_stream);
    if (err != cudaSuccess) {
      fprintf(stderr, "Stream sync error after thrust::sequence: %s\n", cudaGetErrorString(err));
      return -1;
    }
    
    // Get raw pointers from thrust vectors
    d_hash_table_ptr = thrust::raw_pointer_cast(d_hash_table_vec.data());
    d_next_point_ptr = thrust::raw_pointer_cast(d_next_point_vec.data());
    d_parent_ptr = thrust::raw_pointer_cast(d_parent_vec.data());
    d_rank_ptr = thrust::raw_pointer_cast(d_rank_vec.data());
    d_cluster_sizes_ptr = thrust::raw_pointer_cast(d_cluster_sizes_vec.data());
    d_root_to_cluster_ptr = thrust::raw_pointer_cast(d_root_to_cluster_vec.data());
    d_cluster_id_map_ptr = thrust::raw_pointer_cast(d_cluster_id_map_vec.data());
    d_num_clusters_temp_ptr = thrust::raw_pointer_cast(d_num_clusters_temp_vec.data());
    d_num_valid_clusters_ptr = thrust::raw_pointer_cast(d_num_valid_clusters_vec.data());
  }
  
  // Initialize labels to -1 (noise)
  cudaMemsetAsync(d_labels, -1, M * sizeof(int), exec_stream);
  
  int threads_per_block = 256;
  int num_blocks = (M + threads_per_block - 1) / threads_per_block;
  
  // ========== Kernel 1: Build Spatial Hash Grid ==========
  float cell_size = cluster_tolerance;  // Use cluster_tolerance as cell size
  
  buildSpatialHashKernel<<<num_blocks, threads_per_block, 0, exec_stream>>>(
      d_xyz, M, cell_size, d_hash_table_ptr, d_next_point_ptr, table_size);
  
  err = cudaGetLastError();
  if (err != cudaSuccess) {
    fprintf(stderr, "Kernel 1 (build hash) error: %s\n", cudaGetErrorString(err));
    return -1;
  }
  
  // ========== Kernel 2: Find Neighbors and Union ==========
  // Run multiple iterations to ensure all connections are found
  // (union-find may need multiple passes for complete connectivity)
  const int MAX_ITERATIONS = 3;
  for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
    findNeighborsAndUnionKernel<<<num_blocks, threads_per_block, 0, exec_stream>>>(
        d_xyz, M, cluster_tolerance, cell_size,
        d_hash_table_ptr, d_next_point_ptr,
        d_parent_ptr, d_rank_ptr, table_size);
    
    err = cudaGetLastError();
    if (err != cudaSuccess) {
      fprintf(stderr, "Kernel 2 (find neighbors) error: %s\n", cudaGetErrorString(err));
      return -1;
    }
  }
  
  // ========== Kernel 3: Compress Labels (Pass 1) ==========
  compressLabelsPass1Kernel<<<num_blocks, threads_per_block, 0, exec_stream>>>(
      d_parent_ptr, M, d_root_to_cluster_ptr, d_num_clusters_temp_ptr);
  
  err = cudaGetLastError();
  if (err != cudaSuccess) {
    fprintf(stderr, "Kernel 3a (compress pass 1) error: %s\n", cudaGetErrorString(err));
    return -1;
  }
  
  // ========== Kernel 3: Compress Labels (Pass 2) ==========
  compressLabelsPass2Kernel<<<num_blocks, threads_per_block, 0, exec_stream>>>(
      d_parent_ptr, d_root_to_cluster_ptr, M, d_labels);
  
  err = cudaGetLastError();
  if (err != cudaSuccess) {
    fprintf(stderr, "Kernel 3b (compress pass 2) error: %s\n", cudaGetErrorString(err));
    return -1;
  }
  
  // ========== Copy num_clusters to host ==========
  // CRITICAL: Synchronize stream before memcpy to ensure all kernels complete
  err = cudaStreamSynchronize(exec_stream);
  if (err != cudaSuccess) {
    fprintf(stderr, "Stream sync error before memcpy: %s\n", cudaGetErrorString(err));
    return -1;
  }
  
  int num_clusters_temp = 0;
  err = cudaMemcpy(&num_clusters_temp, d_num_clusters_temp_ptr, sizeof(int),
                   cudaMemcpyDeviceToHost);
  if (err != cudaSuccess) {
    fprintf(stderr, "Memcpy error: %s\n", cudaGetErrorString(err));
    return -1;
  }
  
  // ========== Kernel 3: Count Cluster Sizes (Pass 2b) ==========
  if (num_clusters_temp > 0) {
    // Reset cluster sizes to 0
    cudaMemsetAsync(d_cluster_sizes_ptr, 0, num_clusters_temp * sizeof(int), exec_stream);
    
    countClusterSizesKernel<<<num_blocks, threads_per_block, 0, exec_stream>>>(
        d_labels, M, d_cluster_sizes_ptr, num_clusters_temp);
    
    err = cudaGetLastError();
    if (err != cudaSuccess) {
      fprintf(stderr, "Kernel 3b2 (count sizes) error: %s\n", cudaGetErrorString(err));
      return -1;
    }
  }
  
  // ========== Kernel 3: Filter by Size (Pass 3) ==========
  if (num_clusters_temp > 0) {
    int num_blocks_clusters = (num_clusters_temp + threads_per_block - 1) / threads_per_block;
    
    filterClustersBySizeKernel<<<num_blocks_clusters, threads_per_block, 0, exec_stream>>>(
        d_cluster_sizes_ptr, num_clusters_temp, d_labels, M,
        min_cluster_size, max_cluster_size,
        d_cluster_id_map_ptr, d_num_valid_clusters_ptr);
    
    err = cudaGetLastError();
    if (err != cudaSuccess) {
      fprintf(stderr, "Kernel 3c (filter by size) error: %s\n", cudaGetErrorString(err));
      return -1;
    }
  }
  
  // ========== Copy final num_clusters to host ==========
  // CRITICAL: Synchronize stream before memcpy to ensure all kernels complete
  // This prevents Thrust vectors from being destroyed while operations are pending
  err = cudaStreamSynchronize(exec_stream);
  if (err != cudaSuccess) {
    fprintf(stderr, "Stream sync error before final memcpy: %s\n", cudaGetErrorString(err));
    return -1;
  }
  
  if (d_num_clusters != nullptr) {
    err = cudaMemcpy(d_num_clusters, d_num_valid_clusters_ptr, sizeof(int),
                     cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
      fprintf(stderr, "Final memcpy error: %s\n", cudaGetErrorString(err));
      return -1;
    }
  }
  
  // Note: Thrust device vectors will be automatically destroyed here
  // but we've already synchronized, so all operations are complete
  
  return 0;
}

