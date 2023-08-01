# cmake_minimum_required(VERSION 3.8)
project(multimodal_object_detection)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

cmake_minimum_required(VERSION 3.5)
# project(bevfusion)

set(CMAKE_BUILD_PYTHON $ENV{USE_Python})
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})

foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  std_msgs
  common_msgs
  geometry_msgs
  # pcl_conversions
  # pcl_ros
  # tf2
  # tf2_geometry_msgs
  CUDA
  )
  find_package(${PACKAGE} REQUIRED)
endforeach()
find_package(Protobuf REQUIRED)

set(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -Wextra -Wall -Wno-missing-field-initializers -Wno-deprecated-declarations -O3 -DENABLE_TEXT_BACKEND_STB")
set(CMAKE_CXX_FLAGS_DEBUG   "-std=c++14 -O0 -g -DENABLE_TEXT_BACKEND_STB")

set(CUDA_NVCC_FLAGS_RELEASE "-Werror=all-warnings -Xcompiler -std=c++14,-Wextra,-Wall,-Wno-deprecated-declarations,-O3 -DENABLE_TEXT_BACKEND_STB")
set(CUDA_NVCC_FLAGS_DEBUG   "-std=c++14 -O0 -g -DENABLE_TEXT_BACKEND_STB")

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} 
  -gencode arch=compute_89,code=compute_89
)

include_directories(
  ${Protobuf_INCLUDE_DIRS}
  $ENV{CUDA_Inc}
  $ENV{TensorRT_Inc}
  $ENV{Python_Inc}
  src/dependencies/stb
  src/dependencies/pybind11/include
  src/libraries/cuOSD/src
  src/libraries/3DSparseConvolution/libspconv/include
  src/cuda-bevfusion/src/common
  src/cuda-bevfusion/src/
)

link_directories(
  $ENV{CUDA_Lib}
  $ENV{TensorRT_Lib}
  src/libraries/3DSparseConvolution/libspconv/lib/${arch}
  src/cuda-bevfusion/build
  $ENV{Python_Lib}
)

file(GLOB_RECURSE CORE_FILES 
  src/cuda-bevfusion/src/bevfusion/*.cu 
  src/cuda-bevfusion/src/bevfusion/*.cpp
  src/cuda-bevfusion/src/common/tensor.cu
  src/cuda-bevfusion/src/common/tensorrt.cpp
)

file(GLOB_RECURSE VISUALIZE_FILES 
  src/libraries/cuOSD/src/*.cpp
  src/libraries/cuOSD/src/*.cu
  src/cuda-bevfusion/src/common/visualize.cu
)

####################### libbevfusion_core.so ##########################
cuda_add_library(bevfusion_core SHARED 
  ${CORE_FILES}
)

target_link_libraries(bevfusion_core
  libcudart.so
  libcublasLt.so
  libnvinfer.so
  libspconv.so
  ${Protobuf_LIBRARIES}
  libnvinfer_plugin.so
)
#######################################################################


####################### bevfusion program #############################
cuda_add_executable(multimodal_object_detection
  # src/cuda-bevfusion/src/main.cpp
  src/bevfusion_node.cpp
  ${VISUALIZE_FILES}
)

target_link_libraries(multimodal_object_detection
  bevfusion_core
  libnvinfer.so
  libspconv.so
  ${Protobuf_LIBRARIES}
  libnvinfer_plugin.so
)
#######################################################################


# ########################## libpybev.so ################################
# if(${CMAKE_BUILD_PYTHON} STREQUAL "ON")
#   cuda_add_library(pybev SHARED
#     cuda-bevfusion/src/python.cpp
#   )

#   target_link_libraries(pybev
#     bevfusion_core
#     libnvinfer.so
#     libspconv.so
#     ${Protobuf_LIBRARIES}
#     libnvinfer_plugin.so
#     $ENV{Python_Soname}
#   )
# endif()
# #######################################################################
set(CMAKE_CXX_STANDARD 17)
# add_executable(multimodal_object_detection src/bevfusion_node.cpp)
target_include_directories(multimodal_object_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(multimodal_object_detection PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS multimodal_object_detection
  DESTINATION lib/${PROJECT_NAME})

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
