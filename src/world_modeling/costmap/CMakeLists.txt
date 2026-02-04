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
project(costmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(world_model_msgs REQUIRED)
find_package(costmap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Shared library: costmap layer plugins + utilities
add_library(costmap_layers SHARED
  src/costmap_utils.cpp
  src/layers/objects_layer.cpp
  src/layers/pointcloud_layer.cpp
  src/layers/virtual_wall_layer.cpp
)

target_include_directories(costmap_layers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(costmap_layers
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_ros::tf2_ros
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${world_model_msgs_TARGETS}
    ${costmap_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
  PRIVATE
    tf2_geometry_msgs::tf2_geometry_msgs
    pluginlib::pluginlib
)

# Shared library: composable CostmapNode
add_library(costmap_node SHARED
  src/costmap_node.cpp
)

target_include_directories(costmap_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(costmap_node
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_ros::tf2_ros
    pluginlib::pluginlib
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
  PRIVATE
    rclcpp_components::component
)

# Standalone executable
add_executable(costmap_exe src/main.cpp)
set_target_properties(costmap_exe PROPERTIES OUTPUT_NAME costmap_node)

target_link_libraries(costmap_exe
  PRIVATE
    costmap_node
)

# pluginlib plugin descriptor
pluginlib_export_plugin_description_file(costmap plugins.xml)

# Install
install(TARGETS costmap_layers costmap_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS costmap_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
