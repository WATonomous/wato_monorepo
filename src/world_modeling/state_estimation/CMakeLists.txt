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
project(state_estimation)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED) # ROS2 build tool
find_package(rclcpp REQUIRED)      # ROS2 C++ package
find_package(std_msgs REQUIRED)      # ROS2 Messages
find_package(nav_msgs REQUIRED)
find_package(interfacing_msgs REQUIRED)

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(state_estimation_lib src/wheel_odometry_core.cpp)
# Indicate to compiler where to search for header files
target_include_directories(state_estimation_lib
  PUBLIC
  include)
# Add ROS2 dependencies required by package
target_link_libraries(state_estimation_lib
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rclcpp_TARGETS}
  ${interfacing_msgs_TARGETS})

# By default tests are built. This can be overridden by modifying the
# colcon build command to include the -DBUILD_TESTING=OFF flag.
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  # Search for dependencies required for building tests + linting
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  # Remove the default C++ and copyright linter
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_copyright)

  # Reinstall cpplint ignoring copyright errors
  ament_cpplint(FILTERS "-legal/copyright")

  ament_lint_auto_find_test_dependencies()

  # Create test executable from source files
  ament_add_gtest(state_estimation_test test/state_estimation_test.cpp)
  # Link to the previously built library to access state_estimation classes and methods
  target_link_libraries(state_estimation_test
    state_estimation_lib
    gtest_main)

  # Copy executable to installation location
  install(TARGETS
    state_estimation_test
    DESTINATION lib/${PROJECT_NAME})
endif()

# Create ROS2 node executable from source files
add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)

# Link to the previously built library to access occupancy classes and methods
target_link_libraries(wheel_odometry_node state_estimation_lib)

# Copy executable to installation location
install(TARGETS
  wheel_odometry_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
