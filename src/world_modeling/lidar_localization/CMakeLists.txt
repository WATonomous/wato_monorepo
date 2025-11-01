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
project(lidar_localization)

# Compiles source files into a library
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.

# DO LATER: Add your source files here once you create them ------------
# add_library(lidar_localization_lib src/lidar_localization_core.cpp)

# target_include_directories(lidar_localization_lib
#   PUBLIC
#   include)
# target_link_libraries(lidar_localization_lib
#   ${std_msgs_TARGETS}
#   ${sensor_msgs_TARGETS}
#   ${nav_msgs_TARGETS}
#   ${geometry_msgs_TARGETS}
#   ${rclcpp_TARGETS}
#   ${tf2_TARGETS}
#   ${tf2_ros_TARGETS})
#----------------------------------------

# By default tests are built This can be overridden by modifying the
# colcon build command to include the -DBUILD_TESTING=OFF flag.
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # DO LATER: Uncomment and modify once you have test files --------------
  # ament_add_gtest(lidar_localization_test test/lidar_localization_test.cpp)
  # target_link_libraries(lidar_localization_test
  #   lidar_localization_lib
  #   gtest_main)
  # install(TARGETS
  #   lidar_localization_test
  #   DESTINATION lib/${PROJECT_NAME}) ----------
endif()

# DO LATER: create node once ready
# add_executable(lidar_localization_node src/lidar_localization_node.cpp)
# target_link_libraries(lidar_localization_node lidar_localization_lib)
# install(TARGETS
#   lidar_localization_node
#   DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

