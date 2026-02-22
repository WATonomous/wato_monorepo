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
cmake_minimum_required(VERSION 3.14)
project(lattice_planning_markers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lattice_planning_msgs REQUIRED)

# ----------------------------
# Component library
# ----------------------------
add_library(lattice_planning_markers_component SHARED
  src/lattice_planning_markers_node.cpp
)

target_include_directories(lattice_planning_markers_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lattice_planning_markers_component
  rclcpp
  rclcpp_components
  geometry_msgs
  visualization_msgs
  nav_msgs
  lattice_planning_msgs
)

# Register component and generate executable
rclcpp_components_register_node(
  lattice_planning_markers_component
  PLUGIN "lattice_planning_markers::LatticePlanningMarkersNode"
  EXECUTABLE lattice_planning_markers_node
)

# ----------------------------
# Install
# ----------------------------
install(
  TARGETS lattice_planning_markers_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
