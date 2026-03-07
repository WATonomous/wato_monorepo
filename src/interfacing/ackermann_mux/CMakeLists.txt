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
project(ackermann_mux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------
# Dependencies
# ----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# ----------------------------
# Component library
# ----------------------------
add_library(ackermann_mux_component SHARED
  src/input_handle.cpp
  src/ackermann_mux_node.cpp
)

target_include_directories(ackermann_mux_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  SYSTEM PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${rclcpp_lifecycle_INCLUDE_DIRS}
    ${lifecycle_msgs_INCLUDE_DIRS}
    ${ackermann_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

target_link_libraries(ackermann_mux_component
  PRIVATE
    ${rclcpp_LIBRARIES}
    ${rclcpp_components_LIBRARIES}
    ${rclcpp_lifecycle_LIBRARIES}
    ${lifecycle_msgs_LIBRARIES}
    ${ackermann_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
)


rclcpp_components_register_node(
  ackermann_mux_component
  PLUGIN "ackermann_mux::AckermannMuxNode"
  EXECUTABLE ackermann_mux_node

)


# ----------------------------
# Install
# ----------------------------
install(
  TARGETS ackermann_mux_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
  add_wato_test(ackermann_mux_test test/ackermann_mux_test.cpp
    LIBRARIES ackermann_mux_component
  )
  target_link_libraries(ackermann_mux_test ${ackermann_msgs_TARGETS})
endif()

ament_package()
