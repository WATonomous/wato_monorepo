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
project(lanelet_viz_markers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)

# Shared marker utilities library
add_library(marker_utils SHARED
  src/marker_utils.cpp
)
target_include_directories(marker_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(marker_utils
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

# MapVizMarkersNode executable
add_executable(map_viz_markers_node
  src/map_viz_markers_node.cpp
)
target_include_directories(map_viz_markers_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(map_viz_markers_node marker_utils)
ament_target_dependencies(map_viz_markers_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  lanelet_msgs
)

# LaneContextMarkersNode executable
add_executable(lane_context_markers_node
  src/lane_context_markers_node.cpp
)
target_include_directories(lane_context_markers_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lane_context_markers_node marker_utils)
ament_target_dependencies(lane_context_markers_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  lanelet_msgs
)

# Install targets
install(TARGETS
  marker_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  map_viz_markers_node
  lane_context_markers_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
endif()

ament_package()
