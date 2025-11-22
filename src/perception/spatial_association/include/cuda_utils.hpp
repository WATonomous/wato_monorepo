#ifndef CUDA_UTILS_HPP
#define CUDA_UTILS_HPP

#include <cuda_runtime.h>
#include <cstdio>

/**
 * CUDA Error Checking Macros
 * 
 * Usage:
 *   CUDA_CHECK(cudaMalloc(&ptr, size));
 *   CUDA_LAUNCH_KERNEL(kernel, grid, block, shared, stream, args...);
 */

#define CUDA_CHECK(call) \
  do { \
    cudaError_t err = (call); \
    if (err != cudaSuccess) { \
      fprintf(stderr, "[CUDA_ERROR] %s:%d: %s\n", __FILE__, __LINE__, \
              cudaGetErrorString(err)); \
      return false; \
    } \
  } while(0)

#define CUDA_CHECK_NORET(call) \
  do { \
    cudaError_t err = (call); \
    if (err != cudaSuccess) { \
      fprintf(stderr, "[CUDA_ERROR] %s:%d: %s\n", __FILE__, __LINE__, \
              cudaGetErrorString(err)); \
    } \
  } while(0)

#define CUDA_CHECK_RETURN(call, retval) \
  do { \
    cudaError_t err = (call); \
    if (err != cudaSuccess) { \
      fprintf(stderr, "[CUDA_ERROR] %s:%d: %s\n", __FILE__, __LINE__, \
              cudaGetErrorString(err)); \
      return retval; \
    } \
  } while(0)

// For kernel launches
#define CUDA_LAUNCH_KERNEL(kernel, grid, block, shared, stream, ...) \
  do { \
    kernel<<<grid, block, shared, stream>>>(__VA_ARGS__); \
    cudaError_t err = cudaGetLastError(); \
    if (err != cudaSuccess) { \
      fprintf(stderr, "[KERNEL_ERROR] %s:%d: %s\n", __FILE__, __LINE__, \
              cudaGetErrorString(err)); \
      return false; \
    } \
  } while(0)

#define CUDA_LAUNCH_KERNEL_RETURN(kernel, grid, block, shared, stream, retval, ...) \
  do { \
    kernel<<<grid, block, shared, stream>>>(__VA_ARGS__); \
    cudaError_t err = cudaGetLastError(); \
    if (err != cudaSuccess) { \
      fprintf(stderr, "[KERNEL_ERROR] %s:%d: %s\n", __FILE__, __LINE__, \
              cudaGetErrorString(err)); \
      return retval; \
    } \
  } while(0)

/**
 * Debug-only assertions and checks
 */
#ifdef DEBUG
  #define CUDA_DEBUG_CHECK(call) CUDA_CHECK(call)
  #define CUDA_ASSERT(condition, message) \
    do { \
      if (!(condition)) { \
        fprintf(stderr, "[CUDA_ASSERT] %s:%d: %s\n", \
                __FILE__, __LINE__, message); \
        return false; \
      } \
    } while(0)
  #define CUDA_ASSERT_RETURN(condition, message, retval) \
    do { \
      if (!(condition)) { \
        fprintf(stderr, "[CUDA_ASSERT] %s:%d: %s\n", \
                __FILE__, __LINE__, message); \
        return retval; \
      } \
    } while(0)
#else
  #define CUDA_DEBUG_CHECK(call) (call)
  #define CUDA_ASSERT(condition, message) ((void)0)
  #define CUDA_ASSERT_RETURN(condition, message, retval) ((void)0)
#endif

/**
 * Helper function to calculate grid size
 */
inline dim3 calculateGridSize(int N, int block_size) {
  return dim3((N + block_size - 1) / block_size);
}

/**
 * Helper function to check CUDA device properties
 */
inline void printCudaDeviceInfo() {
  int device;
  cudaGetDevice(&device);
  cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, device);
  
  printf("CUDA Device: %s\n", prop.name);
  printf("  Compute Capability: %d.%d\n", prop.major, prop.minor);
  printf("  Multiprocessors: %d\n", prop.multiProcessorCount);
  printf("  Max Threads Per Block: %d\n", prop.maxThreadsPerBlock);
  printf("  Shared Memory Per Block: %zu KB\n", prop.sharedMemPerBlock / 1024);
  printf("  Global Memory: %.2f GB\n", prop.totalGlobalMem / (1024.0 * 1024.0 * 1024.0));
}

#endif // CUDA_UTILS_HPP

