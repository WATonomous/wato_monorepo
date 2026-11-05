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
project(fault_collector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(wato_fault_msgs REQUIRED)

# ----------------------------
# Component library
# ----------------------------
add_library(${PROJECT_NAME}_component SHARED
  src/fault_collector_core.cpp
  src/fault_collector_node.cpp
)

target_include_directories(${PROJECT_NAME}_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_component
  PUBLIC
    ${wato_fault_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${rclcpp_components_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "fault_collector::FaultCollectorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# ----------------------------
# Install
# ----------------------------
install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TARGET ${PROJECT_NAME}_node)
  install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)
  add_wato_test(test_fault_collector_core test/test_fault_collector_core.cpp
    LIBRARIES ${PROJECT_NAME}_component
  )
endif()

ament_package()
