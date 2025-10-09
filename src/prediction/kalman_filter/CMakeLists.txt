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
project(kalman_filter)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- General ROS 2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ---- Msg Packages
find_package(common_msgs REQUIRED)

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(kalman_filter_lib
  src/utils.cpp
)

# Indicate to compiler where to search for header files
target_include_directories(kalman_filter_lib
  PUBLIC
  include)

# Add ROS2 dependencies required by package
ament_target_dependencies(kalman_filter_lib
  rclcpp
  common_msgs
  )

# # By default tests are built. This can be overridden by modifying the
# # colcon build command to include the -DBUILD_TESTING=OFF flag.
# option(BUILD_TESTING "Build tests" ON)
# if(BUILD_TESTING)
#   # Search for dependencies required for building tests + linting
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_lint_common REQUIRED)
#
#   # Remove the default C++ and copyright linter
#   list(APPEND AMENT_LINT_AUTO_EXCLUDE
#     ament_cmake_cpplint
#     ament_cmake_copyright)
#
#   # Reinstall cpplint ignoring copyright errors
#   ament_cpplint(FILTERS "-legal/copyright")
#
#   ament_lint_auto_find_test_dependencies()
#
#   # Create test executable from source files
#   ament_add_gtest(sample_test test/sample_test.cpp)
#   # Link to the previously built library to access Sample classes and methods
#   target_link_libraries(sample_test
#     kalman_filter_lib
#     gtest_main)
#
#   # Copy executable to installation location
#   install(TARGETS
#   sample_test
#     DESTINATION lib/${PROJECT_NAME})
# endif()

# Create ROS2 node executable from source files
add_executable(kalman_filter src/kalman_filter.cpp)
# Link to the previously built library to access Sample classes and methods
target_link_libraries(kalman_filter kalman_filter_lib)

# Copy executable to installation location
install(TARGETS
kalman_filter
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
