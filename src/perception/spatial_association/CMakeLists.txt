cmake_minimum_required(VERSION 3.8)
project(spatial_association)

# 1. Setup Compilers and Standards
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON) # Required for NVCC + GCC compatibility

# Enable CUDA Language
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Set CUDA architectures globally (can be overridden per target)
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES "75;80;86;72;87" CACHE STRING "CUDA architectures")
endif()

# Note: CUDA compile options are set per-target, not globally, to avoid conflicts

# 2. Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport QUIET)  # Optional - not currently used
find_package(pcl_conversions REQUIRED)
find_package(camera_object_detection_msgs REQUIRED)

# Find PCL
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find CUDA Toolkit (Modern CMake approach)
find_package(CUDAToolkit REQUIRED)

# 3. Define GPU Pipeline Library (CUDA)
# -------------------------------------------------------------------
add_library(gpu_pipeline SHARED
  src/gpu_pipeline.cu
  src/gpu_voxel_downsample.cu
  src/gpu_euclidean_clustering.cu
  src/gpu_cluster_stats.cu
)

# Set CUDA architectures for this target (inherits from global if not set)
set_property(TARGET gpu_pipeline PROPERTY CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")

# Set CUDA properties for the library
set_property(TARGET gpu_pipeline PROPERTY CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET gpu_pipeline PROPERTY CUDA_RESOLVE_DEVICE_SYMBOLS ON)

# Set CUDA compiler flags for this target using target_compile_options instead
target_compile_options(gpu_pipeline PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
  $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-std=gnu++17>
  $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-pedantic>
)

target_include_directories(gpu_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link CUDA Runtime (cudart) to gpu_pipeline
# This automatically propagates cuda_runtime.h to anything linking gpu_pipeline
target_link_libraries(gpu_pipeline PUBLIC
  CUDA::cudart
)

# 4. Define Projection Utils Library (C++)
# -------------------------------------------------------------------
add_library(projection_utils SHARED
  src/projection_utils.cpp
  src/orientation_utils.cpp
)

target_include_directories(projection_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# Call ament_target_dependencies FIRST (uses plain signature internally)
ament_target_dependencies(projection_utils
  rclcpp
  sensor_msgs
  vision_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  camera_object_detection_msgs
)

# Then use plain signature for additional libraries (must match ament_target_dependencies style)
target_link_libraries(projection_utils
  gpu_pipeline
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 5. Define ROS Node
# -------------------------------------------------------------------
add_executable(spatial_association_node src/spatial_association.cpp)

target_include_directories(spatial_association_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# Call ament_target_dependencies FIRST (uses plain signature internally)
ament_target_dependencies(spatial_association_node
  rclcpp
  sensor_msgs
  vision_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  pcl_conversions
  camera_object_detection_msgs
)

# Then use plain signature for additional libraries (must match ament_target_dependencies style)
target_link_libraries(spatial_association_node
  projection_utils
  gpu_pipeline
  ${PCL_LIBRARIES}
)

# 6. Install Rules
# -------------------------------------------------------------------
install(TARGETS 
  gpu_pipeline 
  projection_utils 
  spatial_association_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()