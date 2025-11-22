/**
 * GPU Pipeline Implementation
 * * Wires together all CUDA kernels: voxel downsampling, clustering, and stats.
 */

 #include "gpu_pipeline.hpp"
 #include "gpu_voxel_downsample.hpp"
 #include "gpu_euclidean_clustering.hpp"
 #include "gpu_cluster_stats.hpp"
 #include "cuda_utils.hpp"
 #include <cuda_runtime.h>
 #include <cstdint> // Required for uint64_t
 #include <vector>
 #include <cstdio>
 #include <limits>
 #include <algorithm>
 #include <cmath>
 
 // ============================================================================
 // Global Context for Buffer Reuse
 // ============================================================================
 
struct GpuPipelineContext {
  // Device memory buffers (pre-allocated, reused across frames)
  float* d_xyz = nullptr;
  float* d_xyz_voxel = nullptr;
  int* d_labels = nullptr;
  DeviceClusterStats d_stats;
  
  // Voxel downsampling temporary buffers (pre-allocated for performance)
  uint64_t* d_voxel_keys = nullptr;     // Voxel hash keys (size capacity_points)
  int* d_voxel_vals = nullptr;          // Point indices (size capacity_points)
  
  // Clustering temporary buffers (pre-allocated for performance)
  int* d_hash_table = nullptr;          // Hash table for spatial hashing
  int* d_next_point = nullptr;          // Next point in hash cell (linked list)
  int* d_parent = nullptr;               // Union-find parent array
  int* d_rank = nullptr;                 // Union-find rank array
  int* d_cluster_sizes = nullptr;       // Cluster size counts
  int* d_root_to_cluster = nullptr;     // Root to cluster ID mapping
  int* d_cluster_id_map = nullptr;      // Cluster ID remapping
  int* d_num_clusters_temp = nullptr;   // Temporary cluster count (device)
  int* d_num_valid_clusters = nullptr;  // Valid cluster count (device)
  
  // Capacity tracking
  int capacity_points = 0;
  int capacity_clusters = 0;
  int capacity_hash_table = 0;  // Hash table size = capacity_points * HASH_TABLE_SIZE_MULTIPLIER
  
  // CUDA streams
  cudaStream_t stream_compute = nullptr;
  cudaStream_t stream_h2d = nullptr;
  cudaStream_t stream_d2h = nullptr;
  
  // Events for synchronization
  cudaEvent_t event_h2d_complete = nullptr;
  cudaEvent_t event_compute_complete = nullptr;
  
  bool initialized = false;
};
 
 static GpuPipelineContext g_ctx;
 
 // ============================================================================
 // Initialization and Cleanup
 // ============================================================================
 
 bool initializeGpuPipeline(int max_points, int max_clusters) {
   if (g_ctx.initialized) {
     // Already initialized, check if we need to reallocate
     if (max_points > g_ctx.capacity_points || max_clusters > g_ctx.capacity_clusters) {
       cleanupGpuPipeline();
     } else {
       return true;  // Already initialized with sufficient capacity
     }
   }
   
   // First, check if CUDA device is available
   int device_count = 0;
   cudaError_t err = cudaGetDeviceCount(&device_count);
   if (err != cudaSuccess) {
     fprintf(stderr, "[GPU_INIT] Failed to get CUDA device count: %s\n", 
             cudaGetErrorString(err));
     return false;
   }
   if (device_count == 0) {
     fprintf(stderr, "[GPU_INIT] No CUDA devices found. Is GPU available? Is container running with --gpus?\n");
     return false;
   }
   fprintf(stderr, "[GPU_INIT] Found %d CUDA device(s)\n", device_count);
   
   // Initialize CUDA device
   err = cudaSetDevice(0);
   if (err != cudaSuccess) {
     fprintf(stderr, "[GPU_INIT] Failed to set CUDA device 0: %s\n", 
             cudaGetErrorString(err));
     return false;
   }
   
   if (max_points <= 0) max_points = 200000;  // Default
   if (max_clusters <= 0) max_clusters = 4000;  // Default
   
   fprintf(stderr, "[GPU_INIT] Allocating device memory: %d points, %d clusters\n", 
           max_points, max_clusters);
   
   // Allocate device memory
   if (cudaMalloc(&g_ctx.d_xyz, max_points * 3 * sizeof(float)) != cudaSuccess) {
     fprintf(stderr, "[GPU_INIT] Failed to allocate d_xyz: %s\n", 
             cudaGetErrorString(cudaGetLastError()));
     return false;
   }
   if (cudaMalloc(&g_ctx.d_xyz_voxel, max_points * 3 * sizeof(float)) != cudaSuccess) {
     fprintf(stderr, "[GPU_INIT] Failed to allocate d_xyz_voxel: %s\n", 
             cudaGetErrorString(cudaGetLastError()));
     cleanupGpuPipeline();
     return false;
   }
   if (cudaMalloc(&g_ctx.d_labels, max_points * sizeof(int)) != cudaSuccess) {
     fprintf(stderr, "[GPU_INIT] Failed to allocate d_labels: %s\n", 
             cudaGetErrorString(cudaGetLastError()));
     cleanupGpuPipeline();
     return false;
   }
   
  // Allocate cluster stats
  if (allocateDeviceClusterStats(g_ctx.d_stats, max_clusters) != 0) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate device cluster stats\n");
    cleanupGpuPipeline();
    return false;
  }
  
  // Allocate voxel downsampling temporary buffers (for performance optimization)
  if (cudaMalloc(&g_ctx.d_voxel_keys, max_points * sizeof(uint64_t)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_voxel_keys: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  if (cudaMalloc(&g_ctx.d_voxel_vals, max_points * sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_voxel_vals: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  
  // Allocate clustering temporary buffers (for performance optimization)
  // Hash table size = max_points * 2 (HASH_TABLE_SIZE_MULTIPLIER)
  int hash_table_size = max_points * 2;
  if (cudaMalloc(&g_ctx.d_hash_table, hash_table_size * sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_hash_table: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  if (cudaMalloc(&g_ctx.d_next_point, max_points * sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_next_point: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  if (cudaMalloc(&g_ctx.d_parent, max_points * sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_parent: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  if (cudaMalloc(&g_ctx.d_rank, max_points * sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_rank: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  if (cudaMalloc(&g_ctx.d_cluster_sizes, max_points * sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_cluster_sizes: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  if (cudaMalloc(&g_ctx.d_root_to_cluster, max_points * sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_root_to_cluster: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  if (cudaMalloc(&g_ctx.d_cluster_id_map, max_points * sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_cluster_id_map: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  if (cudaMalloc(&g_ctx.d_num_clusters_temp, sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_num_clusters_temp: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  if (cudaMalloc(&g_ctx.d_num_valid_clusters, sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[GPU_INIT] Failed to allocate d_num_valid_clusters: %s\n", 
            cudaGetErrorString(cudaGetLastError()));
    cleanupGpuPipeline();
    return false;
  }
  
  g_ctx.capacity_hash_table = hash_table_size;
  
  // Create CUDA streams
   if (cudaStreamCreate(&g_ctx.stream_compute) != cudaSuccess) {
     fprintf(stderr, "[GPU_INIT] Failed to create compute stream: %s\n", 
             cudaGetErrorString(cudaGetLastError()));
     cleanupGpuPipeline();
     return false;
   }
   if (cudaStreamCreate(&g_ctx.stream_h2d) != cudaSuccess) {
     fprintf(stderr, "[GPU_INIT] Failed to create h2d stream: %s\n", 
             cudaGetErrorString(cudaGetLastError()));
     cleanupGpuPipeline();
     return false;
   }
   if (cudaStreamCreate(&g_ctx.stream_d2h) != cudaSuccess) {
     fprintf(stderr, "[GPU_INIT] Failed to create d2h stream: %s\n", 
             cudaGetErrorString(cudaGetLastError()));
     cleanupGpuPipeline();
     return false;
   }
   
   // Create events
   if (cudaEventCreate(&g_ctx.event_h2d_complete) != cudaSuccess) {
     fprintf(stderr, "[GPU_INIT] Failed to create h2d event: %s\n", 
             cudaGetErrorString(cudaGetLastError()));
     cleanupGpuPipeline();
     return false;
   }
   if (cudaEventCreate(&g_ctx.event_compute_complete) != cudaSuccess) {
     fprintf(stderr, "[GPU_INIT] Failed to create compute event: %s\n", 
             cudaGetErrorString(cudaGetLastError()));
     cleanupGpuPipeline();
     return false;
   }
   
   g_ctx.capacity_points = max_points;
   g_ctx.capacity_clusters = max_clusters;
   g_ctx.initialized = true;
   
   fprintf(stderr, "[GPU_INIT] GPU pipeline initialized successfully\n");
   return true;
 }
 
 void cleanupGpuPipeline() {
   if (!g_ctx.initialized) return;
   
   // Destroy events
   if (g_ctx.event_h2d_complete) {
     cudaEventDestroy(g_ctx.event_h2d_complete);
     g_ctx.event_h2d_complete = nullptr;
   }
   if (g_ctx.event_compute_complete) {
     cudaEventDestroy(g_ctx.event_compute_complete);
     g_ctx.event_compute_complete = nullptr;
   }
   
   // Destroy streams
   if (g_ctx.stream_compute) {
     cudaStreamDestroy(g_ctx.stream_compute);
     g_ctx.stream_compute = nullptr;
   }
   if (g_ctx.stream_h2d) {
     cudaStreamDestroy(g_ctx.stream_h2d);
     g_ctx.stream_h2d = nullptr;
   }
   if (g_ctx.stream_d2h) {
     cudaStreamDestroy(g_ctx.stream_d2h);
     g_ctx.stream_d2h = nullptr;
   }
   
  // Free device memory
  cudaFree(g_ctx.d_xyz);
  cudaFree(g_ctx.d_xyz_voxel);
  cudaFree(g_ctx.d_labels);
  freeDeviceClusterStats(g_ctx.d_stats);
  
  // Free voxel downsampling temporary buffers
  cudaFree(g_ctx.d_voxel_keys);
  cudaFree(g_ctx.d_voxel_vals);
  
  // Free clustering temporary buffers
  cudaFree(g_ctx.d_hash_table);
  cudaFree(g_ctx.d_next_point);
  cudaFree(g_ctx.d_parent);
  cudaFree(g_ctx.d_rank);
  cudaFree(g_ctx.d_cluster_sizes);
  cudaFree(g_ctx.d_root_to_cluster);
  cudaFree(g_ctx.d_cluster_id_map);
  cudaFree(g_ctx.d_num_clusters_temp);
  cudaFree(g_ctx.d_num_valid_clusters);
  
  g_ctx.d_xyz = nullptr;
  g_ctx.d_xyz_voxel = nullptr;
  g_ctx.d_labels = nullptr;
  g_ctx.d_voxel_keys = nullptr;
  g_ctx.d_voxel_vals = nullptr;
  g_ctx.d_hash_table = nullptr;
  g_ctx.d_next_point = nullptr;
  g_ctx.d_parent = nullptr;
  g_ctx.d_rank = nullptr;
  g_ctx.d_cluster_sizes = nullptr;
  g_ctx.d_root_to_cluster = nullptr;
  g_ctx.d_cluster_id_map = nullptr;
  g_ctx.d_num_clusters_temp = nullptr;
  g_ctx.d_num_valid_clusters = nullptr;
  g_ctx.capacity_points = 0;
  g_ctx.capacity_clusters = 0;
  g_ctx.capacity_hash_table = 0;
  g_ctx.initialized = false;
 }
 
 // ============================================================================
 // Main GPU Pipeline Function
 // ============================================================================
 
 bool runGpuPipeline(
     const float* h_xyz,
     int N,
     const GPUParams& params,
     std::vector<int>& h_labels_out,
     std::vector<GPUClusterStats>& clusters_out) {
   
   #ifdef DEBUG
   CUDA_ASSERT(h_xyz != nullptr, "h_xyz is null");
   CUDA_ASSERT(N > 0, "N must be positive");
   CUDA_ASSERT(params.voxel_leaf_size_x > 0.0f, "voxel_leaf_size_x must be positive");
   CUDA_ASSERT(params.min_cluster_size > 0, "min_cluster_size must be positive");
   CUDA_ASSERT(params.max_cluster_size >= params.min_cluster_size, 
               "max_cluster_size must be >= min_cluster_size");
   #endif
   
   if (!g_ctx.initialized) {
     // Auto-initialize with default capacity
     if (!initializeGpuPipeline(N, N / 50)) {  // Estimate clusters
       return false;
     }
   }
   
  // Check capacity
  if (N > g_ctx.capacity_points) {
    // Reallocate with 20% headroom
    int new_capacity = static_cast<int>(N * 1.2f);
    if (!initializeGpuPipeline(new_capacity, new_capacity / 50)) {
      return false;
    }
  }
   
   // ========== STEP 1: Copy input to device (async) ==========
   CUDA_CHECK(cudaMemcpyAsync(g_ctx.d_xyz, h_xyz, N * 3 * sizeof(float),
                              cudaMemcpyHostToDevice, g_ctx.stream_h2d));
   
   // Record event when copy completes
   CUDA_CHECK(cudaEventRecord(g_ctx.event_h2d_complete, g_ctx.stream_h2d));
   
   // Wait for input copy before starting computation
   CUDA_CHECK(cudaStreamWaitEvent(g_ctx.stream_compute, g_ctx.event_h2d_complete));
   
    // ========== STEP 2: Voxel Downsampling ==========
    float voxel_size = params.voxel_leaf_size_x;  // Use x size (assume uniform)
    // Use pre-allocated buffers from context for performance
    int M = runVoxelDownsample(g_ctx.d_xyz, N, voxel_size, 
                              g_ctx.d_xyz_voxel, g_ctx.stream_compute,
                              g_ctx.d_voxel_keys, g_ctx.d_voxel_vals);
   
   if (M < 0) {
     fprintf(stderr, "Voxel downsampling failed\n");
     return false;
   }
   
  if (M == 0) {
    // No points after downsampling
    h_labels_out.clear();
    clusters_out.clear();
    return true;
  }
  
  // Note: Hash table capacity check is not needed here because:
  // - initializeGpuPipeline already sizes hash_table = max_points * 2
  // - M (downsampled points) is always <= N (original points)
  // - So checking N against capacity_points automatically guarantees hash table is large enough
  
  // ========== STEP 3: Euclidean Clustering ==========
    int num_clusters = 0;
    // Use pre-allocated buffers from context for performance
    int result = runGpuClustering(
        g_ctx.d_xyz_voxel, M,
        params.cluster_tolerance,
        params.min_cluster_size,
        params.max_cluster_size,
        g_ctx.d_labels,
        &num_clusters,
        g_ctx.stream_compute,
        // Pre-allocated buffers (performance optimization)
        g_ctx.d_hash_table,
        g_ctx.d_next_point,
        g_ctx.d_parent,
        g_ctx.d_rank,
        g_ctx.d_cluster_sizes,
        g_ctx.d_root_to_cluster,
        g_ctx.d_cluster_id_map,
        g_ctx.d_num_clusters_temp,
        g_ctx.d_num_valid_clusters);
   
   if (result != 0) {
     fprintf(stderr, "GPU clustering failed\n");
     return false;
   }
   
   if (num_clusters == 0) {
     // No valid clusters
     h_labels_out.resize(N, -1);  // All points are noise
     clusters_out.clear();
     return true;
   }
   
   // Check if we need to reallocate cluster stats
   if (num_clusters > g_ctx.capacity_clusters) {
     int new_capacity = static_cast<int>(num_clusters * 1.2f);
     freeDeviceClusterStats(g_ctx.d_stats);
     if (allocateDeviceClusterStats(g_ctx.d_stats, new_capacity) != 0) {
       return false;
     }
     g_ctx.capacity_clusters = new_capacity;
   }
   
   // ========== STEP 4: Cluster Statistics ==========
   runGpuClusterStats(
       g_ctx.d_xyz_voxel,  // Use downsampled points
       g_ctx.d_labels,     // Labels for downsampled points
       M,                  // Number of downsampled points
       num_clusters,
       g_ctx.d_stats,
       g_ctx.stream_compute);
   
   // ========== STEP 5: Map labels back to original N points ==========
   // Strategy: For each original point, compute its voxel key, then find the
   // corresponding downsampled point with that voxel key and copy its label.
   
   // First, synchronize to ensure clustering is complete
   CUDA_CHECK(cudaStreamSynchronize(g_ctx.stream_compute));
   
   // Helper function to compute voxel key (matches voxel downsampling hash function)
   // We use uint64_t for the key
   auto computeVoxelKey = [](float x, float y, float z, float voxel_size) -> uint64_t {
     const float X_MIN = -100.0f, X_MAX = 100.0f;
     const float Y_MIN = -100.0f, Y_MAX = 100.0f;
     const float Z_MIN = -5.0f, Z_MAX = 5.0f;
     const int MAX_VOXEL_DIM = 10000;
     
     x = std::fmax(X_MIN, std::fmin(X_MAX, x));
     y = std::fmax(Y_MIN, std::fmin(Y_MAX, y));
     z = std::fmax(Z_MIN, std::fmin(Z_MAX, z));
     
     int ix = static_cast<int>((x - X_MIN) / voxel_size);
     int iy = static_cast<int>((y - Y_MIN) / voxel_size);
     int iz = static_cast<int>((z - Z_MIN) / voxel_size);
     
     ix = std::max(0, std::min(MAX_VOXEL_DIM - 1, ix));
     iy = std::max(0, std::min(MAX_VOXEL_DIM - 1, iy));
     iz = std::max(0, std::min(MAX_VOXEL_DIM - 1, iz));
     
     uint64_t key = 0;
     key |= (static_cast<uint64_t>(ix) & 0x1FFFFF) << 43;
     key |= (static_cast<uint64_t>(iy) & 0x1FFFFF) << 22;
     key |= (static_cast<uint64_t>(iz) & 0x3FFFFF);
     return key;
   };
   
   // FIX: Removed 'float' to avoid redeclaration error
   voxel_size = params.voxel_leaf_size_x;
   
   // Copy downsampled labels to host first (synchronously)
   std::vector<int> labels_downsampled(M);
   CUDA_CHECK(cudaMemcpy(labels_downsampled.data(), g_ctx.d_labels, M * sizeof(int),
                         cudaMemcpyDeviceToHost));
   
   // Copy downsampled points to host to compute their voxel keys
   std::vector<float> xyz_downsampled(M * 3);
   CUDA_CHECK(cudaMemcpy(xyz_downsampled.data(), g_ctx.d_xyz_voxel, M * 3 * sizeof(float),
                         cudaMemcpyDeviceToHost));
   
   // Create a simple vector-based mapping: voxel_key -> label
   std::vector<std::pair<uint64_t, int> > voxel_to_label;
   voxel_to_label.reserve(M);
   for (int i = 0; i < M; ++i) {
     float x = xyz_downsampled[i * 3 + 0];
     float y = xyz_downsampled[i * 3 + 1];
     float z = xyz_downsampled[i * 3 + 2];
     uint64_t key = computeVoxelKey(x, y, z, voxel_size);
     voxel_to_label.push_back(std::make_pair(key, labels_downsampled[i]));
   }
   
   // PERFORMANCE OPTIMIZATION: Sort by key for Binary Search
   std::sort(voxel_to_label.begin(), voxel_to_label.end(), 
             [](const std::pair<uint64_t, int>& a, const std::pair<uint64_t, int>& b) {
               return a.first < b.first;
             });
   
   // Now map labels to all original points using Binary Search
   h_labels_out.resize(N, -1);
   for (int i = 0; i < N; ++i) {
     float x = h_xyz[i * 3 + 0];
     float y = h_xyz[i * 3 + 1];
     float z = h_xyz[i * 3 + 2];
     uint64_t key = computeVoxelKey(x, y, z, voxel_size);
     
     // Binary search for the key (O(log M))
     auto it = std::lower_bound(voxel_to_label.begin(), voxel_to_label.end(), 
                                std::make_pair(key, -1),
                                [](const std::pair<uint64_t, int>& a, const std::pair<uint64_t, int>& b) {
                                  return a.first < b.first;
                                });
     
     // Check if we actually found the key
     if (it != voxel_to_label.end() && it->first == key) {
       h_labels_out[i] = it->second;
     } else {
       h_labels_out[i] = -1;  // Noise
     }
   }
   
   // ========== STEP 6: Copy cluster stats to host ==========
   std::vector<HostClusterStats> h_stats;
   copyClusterStatsToHost(g_ctx.d_stats, num_clusters, h_stats, g_ctx.stream_compute);
   
   // Record compute completion
   CUDA_CHECK(cudaEventRecord(g_ctx.event_compute_complete, g_ctx.stream_compute));
   
   // Wait for compute before copying output
   CUDA_CHECK(cudaStreamWaitEvent(g_ctx.stream_d2h, g_ctx.event_compute_complete));
   
   // Synchronize to ensure all operations complete
   CUDA_CHECK(cudaStreamSynchronize(g_ctx.stream_compute));
   CUDA_CHECK(cudaStreamSynchronize(g_ctx.stream_d2h));
   
   // Convert HostClusterStats to GPUClusterStats
   clusters_out.clear();
   clusters_out.reserve(h_stats.size());
   for (const auto& h_stat : h_stats) {
     GPUClusterStats gpu_stat;
     gpu_stat.centroid_x = h_stat.centroid_x;
     gpu_stat.centroid_y = h_stat.centroid_y;
     gpu_stat.centroid_z = h_stat.centroid_z;
     gpu_stat.min_x = h_stat.min_x;
     gpu_stat.max_x = h_stat.max_x;
     gpu_stat.min_y = h_stat.min_y;
     gpu_stat.max_y = h_stat.max_y;
     gpu_stat.min_z = h_stat.min_z;
     gpu_stat.max_z = h_stat.max_z;
     gpu_stat.num_points = h_stat.num_points;
     clusters_out.push_back(gpu_stat);
   }
   
   return true;
 }