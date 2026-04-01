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
project(spatial_association)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Messages
find_package(deep_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# TF2
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# PCL
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Other
find_package(OpenCV REQUIRED)

add_library(projection_utils
  utils/src/projection_utils.cpp
  utils/src/cluster_box_utils.cpp
  utils/src/hungarian.cpp
)

target_include_directories(projection_utils
  PRIVATE
    utils/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
)

target_link_libraries(projection_utils
  PUBLIC
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  PRIVATE
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${pcl_conversions_TARGETS}
)

add_library(spatial_association_core
  src/spatial_association_core.cpp
)

target_include_directories(spatial_association_core
  PRIVATE
    include
    utils/include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(spatial_association_core
  PRIVATE
    projection_utils
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
  PUBLIC
    ${PCL_LIBRARIES}
)


# Lifecycle component (shared library for loadable component or linking into standalone exe)
add_library(spatial_association_node SHARED
  src/spatial_association.cpp
)

target_include_directories(spatial_association_node
  PRIVATE
    include
    utils/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
)

target_link_libraries(spatial_association_node
  PRIVATE
    spatial_association_core
    projection_utils
    rclcpp_components::component
    ${deep_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${pcl_conversions_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

rclcpp_components_register_nodes(spatial_association_node
  "wato::perception::spatial_association::SpatialAssociationNode"
)

# Standalone executable (needs include dirs for headers pulled in by spatial_association.hpp)
add_executable(spatial_association_exe src/main.cpp)
set_target_properties(spatial_association_exe PROPERTIES OUTPUT_NAME spatial_association_node)
target_include_directories(spatial_association_exe PRIVATE
  include
  utils/include
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)
target_link_libraries(spatial_association_exe PRIVATE
  spatial_association_node
  ${deep_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${pcl_conversions_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

# Install
install(TARGETS
  spatial_association_node
  spatial_association_core
  projection_utils
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS spatial_association_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY utils/include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
)

ament_package()
