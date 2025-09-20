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
project(localization)

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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)

find_package(common_msgs REQUIRED)

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(localization_lib src/wheel_odometry.cpp)
# Indicate to compiler where to search for header files
target_include_directories(localization_lib
  PUBLIC
  include)
# Add ROS2 dependencies required by package
ament_target_dependencies(localization_lib
  std_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  robot_localization
  common_msgs)

# Create ROS2 node executable from source files

#wheel_odometry is the name of the node which we can run
add_executable(wheel_odometry src/wheel_odometry.cpp)

# Link to the previously built library to access occupancy classes and methods
target_link_libraries(wheel_odometry localization_lib)

# Copy executable to installation location
install(TARGETS
  wheel_odometry
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
