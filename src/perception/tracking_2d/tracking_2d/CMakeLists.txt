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
project(tracking_2d)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(BYTETRACK_DIR "/home/k279huan/ByteTrack-cpp-3d")
# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bytetrack_cpp_vendor REQUIRED)

# Add the executable for the tracking_2d node
add_executable(tracking_2d_node src/tracking_2d.cpp)

# Include directories
target_include_directories(tracking_2d_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    # ${BYTETRACK_DIR}/include
    # ${EIGEN3_INCLUDE_DIR}
    ${tf2_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(tracking_2d_node
    PRIVATE
    # ${BYTETRACK_DIR}/build/libbytetrack.so
    bytetrack_cpp_vendor::bytetrack
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${tf2_LIBRARIES}
    ${tf2_ros_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    # Eigen3::Eigen
    ${OpenCV_LIBS}
)

# Install the executable
install(TARGETS tracking_2d_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_package()
