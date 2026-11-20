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
cmake_minimum_required(VERSION 3.17)
project(prediction_ml)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# AUTO builds the TensorRT backend when TensorRT and the CUDA runtime are found (the world
# modeling image provides both in the MTR engine venv); ON requires them; OFF never builds it.
set(PREDICTION_ML_ENABLE_TENSORRT "AUTO" CACHE STRING "Build the TensorRT MTR backend (AUTO, ON, OFF)")
set_property(CACHE PREDICTION_ML_ENABLE_TENSORRT PROPERTY STRINGS AUTO ON OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(world_model_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(prediction_ml_lib
  src/fallback_prediction.cpp
  src/null_backend.cpp
  src/scene_builder.cpp
  src/output_converter.cpp
  src/mtr_runtime.cpp
)

set(PREDICTION_ML_BUILD_TENSORRT OFF)
if(PREDICTION_ML_ENABLE_TENSORRT STREQUAL "AUTO" OR PREDICTION_ML_ENABLE_TENSORRT)
  # Besides system installs, search the MTR engine venv: its pip wheels ship libnvinfer.so.10 and
  # libcudart.so.12 (versioned names only), and the image adds matching headers under tensorrt/.
  set(_mtr_venv "$ENV{MTR_ENGINE_VENV}")
  if(NOT _mtr_venv)
    set(_mtr_venv /opt/watonomous/mtr-engine-venv)
  endif()
  file(GLOB _mtr_site_packages LIST_DIRECTORIES true "${_mtr_venv}/lib/python3*/site-packages")

  find_path(
    TENSORRT_INCLUDE_DIR
    NAMES NvInfer.h
    HINTS
      $ENV{TENSORRT_ROOT}
      ${_mtr_venv}/tensorrt
      /usr
      /usr/local/TensorRT
    PATH_SUFFIXES
      include
      include/x86_64-linux-gnu
      include/aarch64-linux-gnu
  )

  set(_tensorrt_lib_hints $ENV{TENSORRT_ROOT} /usr /usr/local/TensorRT)
  set(_cudart_include_hints)
  set(_cudart_lib_hints)
  foreach(_site IN LISTS _mtr_site_packages)
    list(APPEND _tensorrt_lib_hints ${_site}/tensorrt_libs)
    list(APPEND _cudart_include_hints ${_site}/nvidia/cuda_runtime/include ${_site}/nvidia/cuda_nvcc/include)
    list(APPEND _cudart_lib_hints ${_site}/nvidia/cuda_runtime/lib)
  endforeach()

  find_library(
    TENSORRT_NVINFER_LIBRARY
    NAMES nvinfer libnvinfer.so.10
    HINTS ${_tensorrt_lib_hints}
    PATH_SUFFIXES
      lib
      lib64
      lib/x86_64-linux-gnu
      lib/aarch64-linux-gnu
  )

  # A full CUDA toolkit (with nvcc) is preferred; otherwise link the CUDA runtime directly, which
  # is all the backend needs and is what the engine venv provides.
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND)
    set(_cudart_target CUDA::cudart)
  else()
    # cuda_runtime_api.h includes crt/host_defines.h, which the CUDA wheels ship in cuda_nvcc.
    find_path(PREDICTION_ML_CUDART_INCLUDE_DIR NAMES cuda_runtime_api.h HINTS ${_cudart_include_hints})
    find_path(PREDICTION_ML_CUDA_CRT_INCLUDE_DIR NAMES crt/host_defines.h HINTS ${_cudart_include_hints})
    find_library(PREDICTION_ML_CUDART_LIBRARY NAMES cudart libcudart.so.12 HINTS ${_cudart_lib_hints})
    if(PREDICTION_ML_CUDART_INCLUDE_DIR AND PREDICTION_ML_CUDA_CRT_INCLUDE_DIR AND PREDICTION_ML_CUDART_LIBRARY)
      add_library(prediction_ml_cudart UNKNOWN IMPORTED)
      set(_cudart_include_dirs ${PREDICTION_ML_CUDART_INCLUDE_DIR} ${PREDICTION_ML_CUDA_CRT_INCLUDE_DIR})
      list(REMOVE_DUPLICATES _cudart_include_dirs)
      set_target_properties(prediction_ml_cudart PROPERTIES
        IMPORTED_LOCATION "${PREDICTION_ML_CUDART_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${_cudart_include_dirs}")
      set(_cudart_target prediction_ml_cudart)
    endif()
  endif()

  set(_tensorrt_missing)
  if(NOT TENSORRT_INCLUDE_DIR)
    list(APPEND _tensorrt_missing "NvInfer.h")
  endif()
  if(NOT TENSORRT_NVINFER_LIBRARY)
    list(APPEND _tensorrt_missing "libnvinfer")
  endif()
  if(NOT _cudart_target)
    list(APPEND _tensorrt_missing "CUDA runtime (cuda_runtime_api.h, crt/host_defines.h, libcudart)")
  endif()

  if(NOT _tensorrt_missing)
    set(PREDICTION_ML_BUILD_TENSORRT ON)
  elseif(PREDICTION_ML_ENABLE_TENSORRT STREQUAL "AUTO")
    message(STATUS "TensorRT MTR backend disabled; not found: ${_tensorrt_missing}")
  else()
    message(
      FATAL_ERROR
      "PREDICTION_ML_ENABLE_TENSORRT=ON but not found: ${_tensorrt_missing}. "
      "Install TensorRT and the CUDA runtime, or set TENSORRT_ROOT / MTR_ENGINE_VENV."
    )
  endif()
endif()

if(PREDICTION_ML_BUILD_TENSORRT)
  message(STATUS "TensorRT include: ${TENSORRT_INCLUDE_DIR}")
  message(STATUS "TensorRT library: ${TENSORRT_NVINFER_LIBRARY}")
  message(STATUS "CUDA runtime: ${_cudart_target}")

  target_sources(
    prediction_ml_lib
    PRIVATE
      src/tensorrt_backend.cpp
  )

  target_compile_definitions(
    prediction_ml_lib
    PRIVATE
      PREDICTION_ML_ENABLE_TENSORRT
  )

  target_include_directories(
    prediction_ml_lib
    SYSTEM
    PRIVATE
      ${TENSORRT_INCLUDE_DIR}
  )

  target_link_libraries(
    prediction_ml_lib
    ${_cudart_target}
    ${TENSORRT_NVINFER_LIBRARY}
  )
endif()

set_target_properties(prediction_ml_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(PREDICTION_ML_BUILD_TENSORRT)
  # The venv's TensorRT and CUDA libraries are not on the loader path; keep their directories in
  # the installed RPATH so the node starts without LD_LIBRARY_PATH.
  set_target_properties(prediction_ml_lib PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()
target_include_directories(prediction_ml_lib PUBLIC include)
target_link_libraries(prediction_ml_lib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${world_model_msgs_TARGETS}
  ${lanelet_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  Threads::Threads
)

add_executable(prediction_ml_node src/prediction_ml_node.cpp)
target_link_libraries(prediction_ml_node prediction_ml_lib)
if(PREDICTION_ML_BUILD_TENSORRT)
  set_target_properties(prediction_ml_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

install(TARGETS prediction_ml_lib
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS prediction_ml_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(rosbag2_cpp REQUIRED)
  find_package(ZLIB REQUIRED)
  ament_add_gtest(test_contract_compiles test/test_contract_compiles.cpp)
  target_link_libraries(test_contract_compiles prediction_ml_lib)
  ament_add_gtest(test_backend test/test_backend.cpp)
  target_link_libraries(test_backend prediction_ml_lib)
  target_compile_definitions(
    test_backend
    PRIVATE
      PREDICTION_ML_CONTRACT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/contract_8_target.yaml")
  if(PREDICTION_ML_BUILD_TENSORRT)
    target_compile_definitions(
      test_backend
      PRIVATE
        PREDICTION_ML_ENABLE_TENSORRT
    )
  endif()
  ament_add_gtest(test_scene_builder test/test_scene_builder.cpp)
  target_link_libraries(test_scene_builder prediction_ml_lib)
  ament_add_gtest(test_runtime test/test_runtime.cpp)
  target_link_libraries(test_runtime prediction_ml_lib)
  ament_add_gtest(
    test_prediction_ml_node
    test/test_prediction_ml_node.cpp
    src/prediction_ml_node.cpp)
  target_compile_definitions(
    test_prediction_ml_node
    PRIVATE
      PREDICTION_ML_DISABLE_MAIN
      PREDICTION_ML_PARAMS_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/params.yaml")
  target_link_libraries(
    test_prediction_ml_node
    prediction_ml_lib
    ${lifecycle_msgs_TARGETS})
  ament_add_gtest(test_output_converter test/test_output_converter.cpp)
  target_link_libraries(test_output_converter prediction_ml_lib)
  ament_add_gtest(test_fallback_prediction test/test_fallback_prediction.cpp)
  target_link_libraries(test_fallback_prediction prediction_ml_lib)
  # tensor_io reads the eval fixtures; only tests link it.
  ament_add_gtest(test_tensor_io test/test_tensor_io.cpp src/tensor_io.cpp)
  target_link_libraries(test_tensor_io prediction_ml_lib ZLIB::ZLIB)
  set_tests_properties(test_tensor_io PROPERTIES
    ENVIRONMENT "PREDICTION_ML_FIXTURE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  ament_add_gtest(test_scene_builder_parity test/test_scene_builder_parity.cpp src/tensor_io.cpp)
  target_link_libraries(
    test_scene_builder_parity
    prediction_ml_lib
    rosbag2_cpp::rosbag2_cpp
    ZLIB::ZLIB)
  set_tests_properties(test_scene_builder_parity PROPERTIES
    ENVIRONMENT "PREDICTION_ML_FIXTURE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
endif()

ament_package()
