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
project(world_model)

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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- Msg Packages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(prediction_msgs REQUIRED)

# ---- Lanelet Packages
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)

# Build library from source files
add_library(world_model_lib
  src/lanelet_handler.cpp
  src/agent_tracker.cpp
  src/traffic_light_tracker.cpp
  src/world_model_node.cpp)

# Include directories
target_include_directories(world_model_lib
  PUBLIC
  include)

# Add ROS2 dependencies
ament_target_dependencies(world_model_lib
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  vision_msgs
  sensor_msgs
  lanelet_msgs
  prediction_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules)

# Create ROS2 node executable
add_executable(world_model_node src/main.cpp)
target_link_libraries(world_model_node world_model_lib)

# Install executable
install(TARGETS
  world_model_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
