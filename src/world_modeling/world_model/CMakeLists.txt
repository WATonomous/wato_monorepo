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
project(world_model)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(prediction_msgs REQUIRED)
find_package(world_model_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/lanelet_handler.cpp
  src/world_model_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${lanelet_msgs_TARGETS}
    ${prediction_msgs_TARGETS}
    ${world_model_msgs_TARGETS}
    lanelet2_core::lanelet2_core
    lanelet2_io::lanelet2_io
    lanelet2_projection::lanelet2_projection
    lanelet2_routing::lanelet2_routing
    lanelet2_traffic_rules::lanelet2_traffic_rules
)

add_executable(world_model_node
  src/world_model_node.cpp
  src/lanelet_handler.cpp
)

target_include_directories(world_model_node PRIVATE include)

target_link_libraries(world_model_node
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${lanelet_msgs_TARGETS}
    ${prediction_msgs_TARGETS}
    ${world_model_msgs_TARGETS}
    lanelet2_core::lanelet2_core
    lanelet2_io::lanelet2_io
    lanelet2_projection::lanelet2_projection
    lanelet2_routing::lanelet2_routing
    lanelet2_traffic_rules::lanelet2_traffic_rules
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS world_model_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)

  add_wato_test(test_world_model
    test/test_world_model.cpp
    LIBRARIES
      ${PROJECT_NAME}_lib
      ${lanelet_msgs_TARGETS}
      ${world_model_msgs_TARGETS}
      ${vision_msgs_TARGETS}
      ${geometry_msgs_TARGETS}
  )
endif()

ament_package()
