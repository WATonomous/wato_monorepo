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
project(lidar_imu_calib)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(lidar_imu_calib SHARED
  src/lidar_imu_calib.cpp
)

target_include_directories(lidar_imu_calib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(lidar_imu_calib
    PRIVATE
    rclcpp_components::component
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
)

# Register component and create executable
rclcpp_compoenents_register_node(lidar_imu_calib_component
  PLUGIN lidar_imu_calib::LidarLidarCalibNode
  EXECUTABLE lidar_imu_calib_node
)

# Install component library and executable
install(TARGETS lidar_imu_calib_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()
