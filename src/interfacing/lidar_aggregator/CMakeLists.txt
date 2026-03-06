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
project(lidar_aggregator)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

add_library(lidar_aggregator SHARED
  src/lidar_aggregator_node.cpp
)

target_include_directories(lidar_aggregator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lidar_aggregator
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

# Link libraries
target_link_libraries(lidar_aggregator
  rclcpp_components::component
  rclcpp::rclcpp
  tf2::tf2
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Register component and create executable
rclcpp_components_register_node(lidar_aggregator
  PLUGIN "lidar_aggregator::LidarAggregatorNode"
  EXECUTABLE lidar_aggregator_node
)

# Install component library and executable
install(TARGETS lidar_aggregator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Unit Testing
# if(BUILD_TESTING)
#   find_package(wato_test REQUIRED)
# endif()

# Export dependencies
ament_package()
