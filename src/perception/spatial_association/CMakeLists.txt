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

# Suppress CMake policy warnings from PCL's FindFLANN module
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(deep_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)

add_library(projection_utils
  utils/src/projection_utils.cpp
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
        tf2_eigen::tf2_eigen
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
    PRIVATE
        ${sensor_msgs_TARGETS}
        ${vision_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${pcl_conversions_TARGETS}
        ${pcl_ros_TARGETS}
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
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
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
    ${pcl_conversions_TARGETS}
    ${pcl_ros_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${camera_object_detection_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    cv_bridge::cv_bridge
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

# Standalone executable (auto configure + activate, then spin)
add_executable(spatial_association_exe src/main.cpp)
set_target_properties(spatial_association_exe PROPERTIES OUTPUT_NAME spatial_association_node)
target_include_directories(spatial_association_exe PRIVATE include)
target_link_libraries(spatial_association_exe PRIVATE spatial_association_node)

# Install the component library and other libraries
install(TARGETS
    spatial_association_node
    spatial_association_core
    projection_utils
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
# Install standalone lifecycle executable
install(TARGETS spatial_association_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
    DESTINATION include
)

# Install utils header files
install(DIRECTORY utils/include/
    DESTINATION include
)


install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
)

ament_package()
