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

option(PREDICTION_ML_ENABLE_TENSORRT "Build the TensorRT MTR backend" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(world_model_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

add_library(prediction_ml_lib
  src/fallback_prediction.cpp
  # >>> PERSON_B sources
  src/null_backend.cpp
  # <<< PERSON_B sources
  # >>> PERSON_A sources
  src/scene_builder.cpp
  # <<< PERSON_A sources
  # >>> PERSON_C sources
  src/output_converter.cpp
  src/mtr_runtime.cpp
  # <<< PERSON_C sources
  src/tensor_io.cpp
)

if(PREDICTION_ML_ENABLE_TENSORRT)
  # target_sources(prediction_ml_lib PRIVATE src/tensorrt_backend.cpp)
  # target_compile_definitions(prediction_ml_lib PRIVATE PREDICTION_ML_ENABLE_TENSORRT)
  find_package(CUDAToolkit REQUIRED)

  find_path(
    TENSORRT_INCLUDE_DIR
    NAMES NvInfer.h
    HINTS
      $ENV{TENSORRT_ROOT}
      /usr
      /usr/local/TensorRT
    PATH_SUFFIXES
      include
      include/x86_64-linux-gnu
      include/aarch64-linux-gnu
  )

  find_library(
    TENSORRT_NVINFER_LIBRARY
    NAMES nvinfer
    HINTS
      $ENV{TENSORRT_ROOT}
      /usr
      /usr/local/TensorRT
    PATH_SUFFIXES
      lib
      lib64
      lib/x86_64-linux-gnu
      lib/aarch64-linux-gnu
  )

  if(NOT TENSORRT_INCLUDE_DIR)
    message(
      FATAL_ERROR
      "PREDICTION_ML_ENABLE_TENSORRT=ON but NvInfer.h was not found. "
      "Install TensorRT or set TENSORRT_ROOT."
    )
  endif()

  if(NOT TENSORRT_NVINFER_LIBRARY)
    message(
      FATAL_ERROR
      "PREDICTION_ML_ENABLE_TENSORRT=ON but libnvinfer was not found. "
      "Install TensorRT or set TENSORRT_ROOT."
    )
  endif()

  message(STATUS "TensorRT include: ${TENSORRT_INCLUDE_DIR}")
  message(STATUS "TensorRT library: ${TENSORRT_NVINFER_LIBRARY}")

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
    PRIVATE
      ${TENSORRT_INCLUDE_DIR}
  )

  target_link_libraries(
    prediction_ml_lib
    CUDA::cudart
    ${TENSORRT_NVINFER_LIBRARY}
  )
endif()

set_target_properties(prediction_ml_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
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
  Threads::Threads
  ZLIB::ZLIB
)

add_executable(prediction_ml_node src/prediction_ml_node.cpp)
target_link_libraries(prediction_ml_node prediction_ml_lib)
add_executable(scene_dump tools/scene_dump.cpp)
target_link_libraries(
  scene_dump
  prediction_ml_lib
  rosbag2_cpp::rosbag2_cpp)

install(TARGETS prediction_ml_lib
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS prediction_ml_node scene_dump DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_contract_compiles test/test_contract_compiles.cpp)
  target_link_libraries(test_contract_compiles prediction_ml_lib)
  # >>> PERSON_B tests
  ament_add_gtest(test_backend test/test_backend.cpp)
  target_link_libraries(test_backend prediction_ml_lib)
  if(PREDICTION_ML_ENABLE_TENSORRT)
    target_compile_definitions(
      test_backend
      PRIVATE
        PREDICTION_ML_ENABLE_TENSORRT
    )
  endif()
  # <<< PERSON_B tests
  # >>> PERSON_A tests
  ament_add_gtest(test_scene_builder test/test_scene_builder.cpp)
  target_link_libraries(test_scene_builder prediction_ml_lib)
  # <<< PERSON_A tests
  # >>> PERSON_C tests
  ament_add_gtest(test_runtime test/test_runtime.cpp)
  target_link_libraries(test_runtime prediction_ml_lib)
  ament_add_gtest(test_output_converter test/test_output_converter.cpp)
  target_link_libraries(test_output_converter prediction_ml_lib)
  # <<< PERSON_C tests
  ament_add_gtest(test_fallback_prediction test/test_fallback_prediction.cpp)
  target_link_libraries(test_fallback_prediction prediction_ml_lib)
  ament_add_gtest(test_tensor_io test/test_tensor_io.cpp)
  target_link_libraries(test_tensor_io prediction_ml_lib)
  set_tests_properties(test_tensor_io PROPERTIES
    ENVIRONMENT "PREDICTION_ML_FIXTURE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  ament_add_gtest(test_scene_builder_parity test/test_scene_builder_parity.cpp)
  target_link_libraries(
    test_scene_builder_parity
    prediction_ml_lib
    rosbag2_cpp::rosbag2_cpp)
  set_tests_properties(test_scene_builder_parity PROPERTIES
    ENVIRONMENT "PREDICTION_ML_FIXTURE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  ament_add_gtest(test_scene_dump test/test_scene_dump.cpp)
  target_link_libraries(
    test_scene_dump
    prediction_ml_lib
    rosbag2_cpp::rosbag2_cpp)
  target_compile_definitions(
    test_scene_dump
    PRIVATE
      SCENE_DUMP_EXECUTABLE="$<TARGET_FILE:scene_dump>")
  add_dependencies(test_scene_dump scene_dump)
  set_tests_properties(test_scene_dump PROPERTIES
    ENVIRONMENT "PREDICTION_ML_FIXTURE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
endif()

ament_package()
