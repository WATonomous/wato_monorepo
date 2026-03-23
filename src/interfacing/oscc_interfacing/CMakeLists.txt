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
project(oscc_interfacing)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roscco_msg REQUIRED)
find_package(std_srvs REQUIRED)

# Build OSCC library subdirectory
add_subdirectory(oscc/api)

# Create library for the node implementation
add_library(oscc_interfacing_lib SHARED
  src/oscc_interfacing.cpp
)

target_include_directories(oscc_interfacing_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/oscc/api/include>
)

target_link_libraries(oscc_interfacing_lib
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  roscco_msg::roscco_msg__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  osccapi_shared_lib
)

add_dependencies(oscc_interfacing_lib osccapi_shared_lib)

# Standalone executable with MultiThreadedExecutor
add_executable(oscc_interfacing_node
  src/main.cpp
)

target_link_libraries(oscc_interfacing_node
  oscc_interfacing_lib
)

# Install executable and library
install(TARGETS oscc_interfacing_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS oscc_interfacing_lib osccapi_shared_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(wato_test QUIET)
#   if(wato_test_FOUND)
#     add_wato_test(oscc_interfacing_test test/oscc_interfacing_test.cpp
#       LIBRARIES oscc_interfacing
#     )
#     target_link_libraries(oscc_interfacing_test
#       ${sensor_msgs_TARGETS}
#       ${std_msgs_TARGETS}
#       roscco_msg::roscco_msg__rosidl_typesupport_cpp
#       std_srvs::std_srvs__rosidl_typesupport_cpp
#     )
#   endif()
# endif()

ament_package()
