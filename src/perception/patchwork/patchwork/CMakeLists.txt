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

# C++ standard & warnings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 & deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(patchworkpp_vendor REQUIRED)

# Component library (for composable node loading)
add_library(patchworkpp_component SHARED
  src/ground_removal_core.cpp
  src/ground_removal_node.cpp
)

target_include_directories(patchworkpp_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(patchworkpp_component
  PRIVATE
  patchworkpp_vendor::ground_seg_cores
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  diagnostic_updater::diagnostic_updater
  Eigen3::Eigen
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
)

# Component registration is handled by RCLCPP_COMPONENTS_REGISTER_NODE macro in source file

# Standalone executable (for backward compatibility)
add_executable(patchworkpp_node
  src/ground_removal_core.cpp
  src/ground_removal_node.cpp
)

target_include_directories(patchworkpp_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(patchworkpp_node
  PRIVATE
  patchworkpp_vendor::ground_seg_cores
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  diagnostic_updater::diagnostic_updater
  Eigen3::Eigen
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
)

# Install both component library and executable
install(TARGETS patchworkpp_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS patchworkpp_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
