cmake_minimum_required(VERSION 3.8)
set(ProjectName "lane_detection")
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
SET(CMAKE_CUDA_COMPILER /usr/local/cuda-12.2/bin/nvcc)
SET(CUDACXX /usr/local/cuda-12.2/bin/nvcc)
project(${ProjectName} LANGUAGES CXX CUDA)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### TensorRT model ###

# Select build system and set compile options
include(${CMAKE_CURRENT_LIST_DIR}/src/lane_detection_tensorrt/common_helper/cmakes/build_setting.cmake)

add_executable(lane_detection src/lane_detection_node.cpp)

add_subdirectory(./src/lane_detection_tensorrt/ultra_fast_lane_detection_v2/image_processor image_processor)
target_include_directories(${ProjectName} PUBLIC ./src/lane_detection_tensorrt/ultra_fast_lane_detection_v2/image_processor)
target_link_libraries(${ProjectName} ImageProcessor)

find_package(OpenCV REQUIRED)
target_include_directories(${ProjectName} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${ProjectName} ${OpenCV_LIBS})

# Copy resouce
add_definitions(-DRESOURCE_DIR="/models/")

### End of TensorRT model ###

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lane_detection_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(CUDA REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
lane_detection
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(lane_detection rclcpp OpenCV std_msgs sensor_msgs cv_bridge image_transport lane_detection_msgs)
ament_package()
