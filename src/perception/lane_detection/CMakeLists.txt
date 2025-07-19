# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.8)
set(ProjectName "lane_detection")
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
set(CMAKE_CUDA_COMPILER /usr/local/cuda-12.2/bin/nvcc)
set(CUDACXX /usr/local/cuda-12.2/bin/nvcc)
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
target_include_directories(${ProjectName} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ./src/lane_detection_tensorrt/ultra_fast_lane_detection_v2/image_processor)
target_link_libraries(${ProjectName} ImageProcessor)

find_package(OpenCV REQUIRED)
target_include_directories(${ProjectName} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${ProjectName} ${OpenCV_LIBS})

# Copy resouce
add_definitions(-DRESOURCE_DIR="/models/")

### End of TensorRT model ###

set(REQUIRED_PACKAGES
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  lane_detection_msgs
  cv_bridge
  image_transport
  CUDA
)

foreach(PKG IN LISTS REQUIRED_PACKAGES)
  find_package(${PKG} REQUIRED)
endforeach()

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
