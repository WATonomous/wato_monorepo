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
project(trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(behaviour_msgs REQUIRED)
find_package(wato_trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(wato_lifecycle_manager REQUIRED)

# Component Library
add_library(${PROJECT_NAME}_component SHARED
  src/trajectory_core.cpp
  src/trajectory_planner_node.cpp
)

target_include_directories(${PROJECT_NAME}_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_component
  PUBLIC
    ${nav_msgs_TARGETS}
    ${wato_trajectory_msgs_TARGETS}
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${rclcpp_components_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${lanelet_msgs_TARGETS}
    ${behaviour_msgs_TARGETS}
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${wato_lifecycle_manager_TARGETS}
)

# Register component and generate executable
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "trajectory_planner::TrajectoryPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS
  ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TARGET ${PROJECT_NAME}_node)
  install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)

  add_wato_test(test_trajectory_core
    test/test_trajectory_core.cpp
    LIBRARIES
      ${PROJECT_NAME}_component
  )
endif()

ament_package()
