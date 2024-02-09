# cmake_minimum_required(VERSION 3.8)
# project(lane_detection)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

cmake_minimum_required(VERSION 3.8)
set(ProjectName "lane_detection")
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
SET(CMAKE_CUDA_COMPILER /usr/local/cuda-12.2/bin/nvcc)
SET(CUDACXX /usr/local/cuda-12.2/bin/nvcc)
project(${ProjectName} LANGUAGES CXX CUDA)

# set(CUDA_NVCC_EXECUTABLE "/usr/local/cuda/bin/nvcc")


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### TensorRT model ###

# Select build system and set compile options
include(${CMAKE_CURRENT_LIST_DIR}/src/tensorrt_model/common_helper/cmakes/build_setting.cmake)

add_executable(lane_detection src/lane_detection_node.cpp)

# Link ImageProcessor module
add_subdirectory(./src/tensorrt_model/ultra_fast_lane_detection_v2/image_processor image_processor)
target_include_directories(${ProjectName} PUBLIC ./src/tensorrt_model/ultra_fast_lane_detection_v2/image_processor)
target_link_libraries(${ProjectName} ImageProcessor)

# For OpenCV
find_package(OpenCV REQUIRED)
target_include_directories(${ProjectName} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${ProjectName} ${OpenCV_LIBS})

# Copy resouce
file(COPY ${CMAKE_CURRENT_LIST_DIR}/src/tensorrt_model/ultra_fast_lane_detection_v2/resource/model DESTINATION ${CMAKE_BINARY_DIR}/resource)
add_definitions(-DRESOURCE_DIR="${CMAKE_BINARY_DIR}/resource/")


### End of TensorRT model ###

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(CUDA REQUIRED)


install(TARGETS
lane_detection
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(lane_detection rclcpp OpenCV std_msgs sensor_msgs cv_bridge image_transport)
ament_package()
