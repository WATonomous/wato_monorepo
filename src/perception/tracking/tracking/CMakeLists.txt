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
project(tracking)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bytetrack_cpp_vendor REQUIRED)

# Add library
add_library(${PROJECT_NAME}_lib SHARED
    src/tracking.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_lib
    PUBLIC
    bytetrack_cpp_vendor::bytetrack
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${std_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${tf2_ros_TARGETS}
    PRIVATE
    rclcpp_components::component
    ${lifecycle_msgs_TARGETS}
    ${tf2_LIBRARIES}
    ${tf2_geometry_msgs_TARGETS}
)

# Add the executable (uncomment if using this package's main)
# add_executable(${PROJECT_NAME}_node src/main.cpp)

# target_link_libraries(${PROJECT_NAME}_node
#     PRIVATE ${PROJECT_NAME}_lib
# )

# Register the component
rclcpp_components_register_nodes(${PROJECT_NAME}_lib
  "TrackingNode"
)

# Install library
install(TARGETS ${PROJECT_NAME}_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install the executable (uncomment if using this package's main)
# install(TARGETS ${PROJECT_NAME}_node RUNTIME
#     DESTINATION lib/${PROJECT_NAME}
# )

# Install launch and config files
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

# Testing option
# option(BUILD_TESTING "Enable testing" OFF)
if(BUILD_TESTING)
  find_package(wato_test REQUIRED)

  add_wato_test(test_tracking
    test/test_tracking.cpp
    LIBRARIES
      ${PROJECT_NAME}_lib
      ${vision_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${tf2_LIBRARIES}
      ${tf2_geometry_msgs_TARGETS}
  )
endif()

# Export dependencies
ament_package()
