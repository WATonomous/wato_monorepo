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
project(eidos_transform)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eidos_msgs REQUIRED)
find_package(pluginlib REQUIRED)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- Core library ----
add_library(eidos_transform_core SHARED
  src/eidos_transform_node.cpp
  src/holonomic_ekf.cpp
)
target_include_directories(eidos_transform_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(eidos_transform_core PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
  ${eidos_msgs_TARGETS}
  pluginlib::pluginlib
  gtsam
)

# ---- Plugin library ----
add_library(eidos_transform_plugins SHARED
  src/holonomic_ekf.cpp
)
target_include_directories(eidos_transform_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(eidos_transform_plugins PUBLIC
  eidos_transform_core
)

# ---- Node executable ----
add_executable(eidos_transform_node src/main.cpp)
target_link_libraries(eidos_transform_node PRIVATE eidos_transform_core)

# ---- Plugin descriptions ----
pluginlib_export_plugin_description_file(eidos_transform ekf_plugins.xml)

# ---- Install ----
install(TARGETS eidos_transform_core eidos_transform_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS eidos_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ekf_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---- Export ----
ament_export_include_directories(include)
ament_export_libraries(eidos_transform_core eidos_transform_plugins)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle nav_msgs geometry_msgs sensor_msgs
  tf2 tf2_ros tf2_geometry_msgs eidos_msgs pluginlib
)

ament_package()
