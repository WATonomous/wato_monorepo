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
project(prediction)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- General ROS 2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ---- Msg Packages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(world_model_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)

# ---- World Model for LaneletHandler
find_package(world_model REQUIRED)

# ---- Lanelet2 for routing graph queries
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_routing REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(prediction_lib
  src/trajectory_predictor.cpp
  src/intent_classifier.cpp
  src/motion_models.cpp
)

set_target_properties(prediction_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(prediction_lib
  PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR})

# Link ROS2 dependencies required by package
target_link_libraries(prediction_lib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${world_model_msgs_TARGETS}
  ${lanelet_msgs_TARGETS}
  world_model::world_model_lib
  lanelet2_core::lanelet2_core
  lanelet2_routing::lanelet2_routing
  Eigen3::Eigen
  )

add_executable(prediction_node src/prediction_node.cpp)
target_link_libraries(prediction_node prediction_lib)

install(TARGETS prediction_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS prediction_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
