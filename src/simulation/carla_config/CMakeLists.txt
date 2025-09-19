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
cmake_minimum_required(VERSION 3.5)
project(carla_config)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(path_planning_msgs REQUIRED)
find_package(embedded_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create ROS2 node executable from source files
add_executable(carla_mpc_bridge src/carla_mpc_bridge.cpp)
add_executable(carla_waypoint_modifier src/carla_waypoint_fix.cpp)
add_executable(carla_image_encoding_conversion src/carla_image_encoding_conversion.cpp)
add_executable(vehicle_status_bridge src/vehicle_status_bridge.cpp)
add_executable(sensor_frame_bridge src/sensor_frame_bridge.cpp)

if(BUILD_TESTING)
  # Search for dependencies required for building tests + linting
  find_package(ament_cmake_gtest REQUIRED)

  # Create library to link cpp file with test
  add_library(mpc_bridge_lib
    src/carla_mpc_bridge.cpp)

  # Add ROS2 dependencies required by package
  ament_target_dependencies(mpc_bridge_lib rclcpp path_planning_msgs embedded_msgs)

  # Create test executable from source files
  ament_add_gtest(mpc_bridge_test test/mpc_bridge_test.cpp)
  # Link to dependencies
  target_link_libraries(mpc_bridge_test mpc_bridge_lib)

  # Copy executable to installation location
  install(TARGETS
    mpc_bridge_test
    DESTINATION lib/${PROJECT_NAME})
endif()

# Link with dependencies
ament_target_dependencies(carla_mpc_bridge rclcpp path_planning_msgs embedded_msgs)
ament_target_dependencies(carla_waypoint_modifier rclcpp nav_msgs)
ament_target_dependencies(carla_image_encoding_conversion rclcpp sensor_msgs)
ament_target_dependencies(vehicle_status_bridge rclcpp carla_msgs common_msgs)
ament_target_dependencies(sensor_frame_bridge rclcpp sensor_msgs tf2_ros geometry_msgs)


# Copy executable to installation location
install(TARGETS
  carla_mpc_bridge
  carla_waypoint_modifier
  carla_image_encoding_conversion
  vehicle_status_bridge
  sensor_frame_bridge
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
