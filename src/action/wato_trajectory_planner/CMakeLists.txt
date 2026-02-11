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
project(wato_trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(wato_trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(wato_lifecycle_manager REQUIRED)

include_directories(include)

# Trajectory Core Library (Math logic)
add_library(${PROJECT_NAME}_core
  src/trajectory_core.cpp
)
target_link_libraries(${PROJECT_NAME}_core
  nav_msgs::nav_msgs__rosidl_generator_c
  nav_msgs::nav_msgs__rosidl_typesupport_introspection_c
  nav_msgs::nav_msgs__rosidl_typesupport_c
  nav_msgs::nav_msgs__rosidl_generator_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_introspection_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_generator_c
  geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_c
  geometry_msgs::geometry_msgs__rosidl_typesupport_c
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  wato_trajectory_msgs::wato_trajectory_msgs__rosidl_generator_c
  wato_trajectory_msgs::wato_trajectory_msgs__rosidl_typesupport_introspection_c
  wato_trajectory_msgs::wato_trajectory_msgs__rosidl_typesupport_c
  wato_trajectory_msgs::wato_trajectory_msgs__rosidl_generator_cpp
  wato_trajectory_msgs::wato_trajectory_msgs__rosidl_typesupport_introspection_cpp
  wato_trajectory_msgs::wato_trajectory_msgs__rosidl_typesupport_cpp
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  nav_msgs
  geometry_msgs
  wato_trajectory_msgs
  tf2_geometry_msgs
)

# Trajectory Planner Node
add_executable(${PROJECT_NAME}_node
  src/trajectory_planner_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  geometry_msgs
  visualization_msgs
  wato_trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  wato_lifecycle_manager
)

install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
