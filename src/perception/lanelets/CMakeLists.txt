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

cmake_minimum_required(VERSION 3.18)
project(lanelets)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(deep_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(semvecnet REQUIRED CONFIG)

add_library(${PROJECT_NAME}_node SHARED
  src/lanelets_core.cpp
  src/lanelets_node.cpp
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node
  PUBLIC
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${deep_msgs_TARGETS}
    lanelet2_core::lanelet2_core
    ${lanelet_msgs_TARGETS}
    message_filters::message_filters
    tf2_ros::tf2_ros
    ${OpenCV_LIBS}
  PRIVATE
    cv_bridge::cv_bridge
    rclcpp_components::component
    tf2_eigen::tf2_eigen
    semvecnet::semvecnet_core
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

rclcpp_components_register_nodes(${PROJECT_NAME}_node
  "wato::perception::lanelets::LaneletsNode"
)

add_executable(${PROJECT_NAME}_exe
  src/main.cpp
)

set_target_properties(${PROJECT_NAME}_exe PROPERTIES
  OUTPUT_NAME lanelets_node
)

target_link_libraries(${PROJECT_NAME}_exe
  PRIVATE
    ${PROJECT_NAME}_node
)

install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp sensor_msgs geometry_msgs deep_msgs lanelet2_core lanelet_msgs message_filters tf2_ros OpenCV
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lanelets_core test/test_lanelets_core.cpp)
  if(TARGET test_lanelets_core)
    target_link_libraries(test_lanelets_core ${PROJECT_NAME}_node)
  endif()
endif()

ament_package()
