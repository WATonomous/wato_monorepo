# SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(pp_infer)
set(PROJECT_NAME pp_infer)
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH )
message( STATUS "Architecture: ${ARCH}" )

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
# find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
 
find_package(CUDA REQUIRED)

set(CUDA_VERSION 11.3)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-${CUDA_VERSION})

SET(CMAKE_BUILD_TYPE "Release")
add_compile_options(-W)
add_compile_options(-std=c++11)

set(SMS 50 52 53 60 61 62 70 72 75 80 86)
foreach(sm ${SMS})
    set(GENCODE ${GENCODE} -gencode arch=compute_${sm},code=sm_${sm})
endforeach()
list(GET SMS -1 LATEST_SM)
set(GENCODE "${GENCODE} -gencode arch=compute_${LATEST_SM},code=compute_${LATEST_SM}")

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}
    -ccbin ${CMAKE_CXX_COMPILER}
    -Xcompiler -DWIN_INTERFACE_CUSTOM
    -Xcompiler -I/usr/${ARCH}-linux-gnu/include/
    -Xlinker -lsocket
    -Xlinker -rpath=/usr/lib/${ARCH}-linux-gnu/
    -Xlinker -rpath=/usr/${ARCH}-linux-gnu/lib/
    -Xlinker -L/usr/lib/${ARCH}-linux-gnu/
    -Xlinker -L/usr/${ARCH}-linux-gnu/lib/
)

set(TENSORRT_INCLUDE_DIRS /usr/include/${ARCH}-linux-gnu/)
set(TENSORRT_LIBRARY_DIRS /usr/lib/${ARCH}-linux-gnu/)

include_directories(
    ${CUDA_INCLUDE_DIRS}
    /usr/local/cuda-11.4/targets/aarch64-linux/include/
    ${TENSORRT_INCLUDE_DIRS}
    /usr/include/aarch64-linux-gnu/
    # ../include/
    src/CUDA-PointPillars/src
)

link_directories(
    ${TENSORRT_LIBRARY_DIRS}
    /usr/lib/${ARCH}-linux-gnu
    /usr/${ARCH}-linux-gnu/lib/
    $ENV{CUDA_Lib}
    $ENV{TensorRT_Lib}
    build
)

file(GLOB_RECURSE SOURCE_FILES
    # src/pointpillar.cpp
    # src/postprocess.cpp
    src/CUDA-PointPillars/src/pointpillar/*.cu
    src/CUDA-PointPillars/src/pointpillar/*.cpp
    src/CUDA-PointPillars/src/common/tensor.cu
    src/CUDA-PointPillars/src/common/tensorrt.cpp

)

file(GLOB_RECURSE CORE_FILES
  src/CUDA-PointPillars/src/pointpillar/*.cu
  src/CUDA-PointPillars/src/pointpillar/*.cpp
  src/CUDA-PointPillars/src/common/tensor.cu
  src/CUDA-PointPillars/src/common/tensorrt.cpp
)

####################### libpointpillar_core.so ##########################
cuda_add_library(pointpillar_core SHARED 
  	${CORE_FILES}
)

target_link_libraries(pointpillar_core
	libnvinfer.so
)
#######################################################################



cuda_add_executable(${PROJECT_NAME} src/pp_inference_node.cpp ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  std_msgs
  vision_msgs
  geometry_msgs
  PCL
  CUDA
  message_filters
  pcl_conversions
  pcl_ros
  visualization_msgs)

target_link_libraries(${PROJECT_NAME}
    libnvinfer.so
    libnvonnxparser.so
    libnvinfer_plugin.so
    pointpillar_core
)


install(TARGETS
	${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
