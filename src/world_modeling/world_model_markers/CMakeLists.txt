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
project(world_model_markers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(world_model_msgs REQUIRED)
find_package(lanelet_markers REQUIRED)

# WorldObjectsMarkersNode executable
add_executable(world_objects_markers_node
  src/world_objects_markers_node.cpp
)
target_include_directories(world_objects_markers_node PRIVATE include)
target_link_libraries(world_objects_markers_node
  PRIVATE
    lanelet_markers::marker_utils
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${world_model_msgs_TARGETS}
)

# DynamicObjectsMarkersNode executable
add_executable(dynamic_objects_markers_node
  src/dynamic_objects_markers_node.cpp
)
target_include_directories(dynamic_objects_markers_node PRIVATE include)
target_link_libraries(dynamic_objects_markers_node
  PRIVATE
    lanelet_markers::marker_utils
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${world_model_msgs_TARGETS}
)

# Install targets
install(TARGETS
  world_objects_markers_node
  dynamic_objects_markers_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
endif()

ament_package()
