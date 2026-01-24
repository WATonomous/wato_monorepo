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
project(behaviour)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ══════════════════════════════════════════════════════════════════════
# Dependencies
# ══════════════════════════════════════════════════════════════════════

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(world_model_msgs REQUIRED)
find_package(behaviour_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ══════════════════════════════════════════════════════════════════════
# Library
# ══════════════════════════════════════════════════════════════════════

add_library(${PROJECT_NAME}_lib SHARED
  src/behaviour_node.cpp
  src/behaviour_tree.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}_lib
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  std_msgs
  std_srvs
  behaviortree_cpp
  behaviortree_ros2
  world_model_msgs
  behaviour_msgs
  geometry_msgs
  nav_msgs
  lanelet_msgs
  tf2_ros
  tf2_geometry_msgs
)

# ══════════════════════════════════════════════════════════════════════
# Executables
# ══════════════════════════════════════════════════════════════════════

add_executable(behaviour_node
  src/behaviour_node.cpp
)

target_include_directories(behaviour_node PRIVATE include)

target_link_libraries(behaviour_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(behaviour_node
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  std_msgs
  std_srvs
  behaviortree_cpp
  behaviortree_ros2
  world_model_msgs
  behaviour_msgs
  geometry_msgs
  nav_msgs
  lanelet_msgs
  tf2_ros
  tf2_geometry_msgs
)

# ══════════════════════════════════════════════════════════════════════
# Install
# ══════════════════════════════════════════════════════════════════════

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  behaviour_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch trees
  DESTINATION share/${PROJECT_NAME}
)

# ══════════════════════════════════════════════════════════════════════
# Testing
# ══════════════════════════════════════════════════════════════════════

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
