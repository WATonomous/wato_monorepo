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
project(patchworkpp)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies first
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(patchworkpp REQUIRED)

# ROS 2 ground removal node
add_library(ground_removal_node SHARED src/ground_removal.cpp)

target_compile_features(ground_removal_node PUBLIC cxx_std_20)

target_include_directories(ground_removal_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(ground_removal_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(ground_removal_node ${PATCHWORKPP_TARGET})
ament_target_dependencies(
  ground_removal_node
  rcutils
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  std_msgs
  eigen3_cmake_module)

rclcpp_components_register_node(ground_removal_node PLUGIN "patchworkpp_ros::GroundRemovalServer" EXECUTABLE patchworkpp_ground_removal_node)

install(TARGETS ground_removal_node LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS patchworkpp_ground_removal_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(Eigen3)
ament_package()
