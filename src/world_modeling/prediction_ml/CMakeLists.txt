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
cmake_minimum_required(VERSION 3.10)
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

add_library(prediction_ml_lib
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
)

if(PREDICTION_ML_ENABLE_TENSORRT)
  target_sources(prediction_ml_lib PRIVATE src/tensorrt_backend.cpp)
  target_compile_definitions(prediction_ml_lib PRIVATE PREDICTION_ML_ENABLE_TENSORRT)
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
)

add_executable(prediction_ml_node src/prediction_ml_node.cpp)
target_link_libraries(prediction_ml_node prediction_ml_lib)

install(TARGETS prediction_ml_lib
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS prediction_ml_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_contract_compiles test/test_contract_compiles.cpp)
  target_link_libraries(test_contract_compiles prediction_ml_lib)
  # >>> PERSON_B tests
  ament_add_gtest(test_backend test/test_backend.cpp)
  target_link_libraries(test_backend prediction_ml_lib)
  # <<< PERSON_B tests
  # >>> PERSON_A tests
  ament_add_gtest(test_scene_builder test/test_scene_builder.cpp)
  target_link_libraries(test_scene_builder prediction_ml_lib)
  # <<< PERSON_A tests
  # >>> PERSON_C tests
  ament_add_gtest(test_runtime test/test_runtime.cpp)
  target_link_libraries(test_runtime prediction_ml_lib)
  # <<< PERSON_C tests
endif()

ament_package()
