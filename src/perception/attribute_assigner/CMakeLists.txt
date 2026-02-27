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
project(attribute_assigner)

# C++ standard & warnings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ---- Core library (image-based attribute logic) ----
add_library(${PROJECT_NAME}_core SHARED
  src/attribute_assigner_core.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    ${vision_msgs_TARGETS}
    ${OpenCV_LIBS}
)

# ---- Node library (lifecycle composable node) ----
add_library(${PROJECT_NAME}_node SHARED
  src/attribute_assigner_node.cpp
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    diagnostic_updater::diagnostic_updater
    message_filters::message_filters
    ${sensor_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${PROJECT_NAME}_core
  PRIVATE
    rclcpp_components::component
    cv_bridge::cv_bridge
    ${diagnostic_msgs_TARGETS}
)

# ---- Executable ----
add_executable(${PROJECT_NAME}_exe
  src/main.cpp
)

set_target_properties(${PROJECT_NAME}_exe PROPERTIES
  OUTPUT_NAME attribute_assigner_node
)

target_include_directories(${PROJECT_NAME}_exe
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_exe
  PRIVATE
    ${PROJECT_NAME}_node
)

# ---- Install ----
install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  TARGETS ${PROJECT_NAME}_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ---- Export ----
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  diagnostic_updater
  diagnostic_msgs
  vision_msgs
  std_msgs
  sensor_msgs
  message_filters
  cv_bridge
  OpenCV
)

ament_package()
