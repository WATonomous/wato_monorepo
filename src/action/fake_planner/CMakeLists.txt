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
project(fake_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviour_msgs REQUIRED)
find_package(wato_trajectory_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Component Library
add_library(${PROJECT_NAME}_nodes SHARED
  src/fake_planner_node.cpp
)

target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_nodes PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${behaviour_msgs_TARGETS}
  ${wato_trajectory_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
)

# Register Component
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "fake_planner::FakePlannerNode"
  EXECUTABLE fake_planner_node
)

# Install
install(TARGETS ${PROJECT_NAME}_nodes
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch maneuvers
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
