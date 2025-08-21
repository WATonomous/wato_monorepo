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
project(patchwork)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sample_msgs REQUIRED)
find_package(camera_object_detection_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED) # Added PCL find_package

add_library(projection_utils
  src/projection_utils.cpp
)

target_include_directories(projection_utils
    PUBLIC
    include
    projection_utils
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} # Corrected PCL includes
)

ament_target_dependencies(projection_utils
    rclcpp
    sample_msgs
    sensor_msgs
    vision_msgs
    visualization_msgs
    camera_object_detection_msgs
    pcl_conversions
    pcl_ros
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    std_msgs
    geometry_msgs
)

# Add the executable for the bbox_2d_3d node
add_executable(bbox_2d_3d_node src/bbox_2d_3d.cpp)

# Include directories
target_include_directories(bbox_2d_3d_node
    PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} # Corrected PCL includes
)

# Link libraries
target_link_libraries(bbox_2d_3d_node
    projection_utils
    ${OpenCV_LIBS}
    tf2::tf2
    ${PCL_LIBRARIES} #added PCL libraries
)

# Add ROS 2 dependencies
ament_target_dependencies(bbox_2d_3d_node
    rclcpp
    sensor_msgs
    geometry_msgs
    vision_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    cv_bridge
    pcl_conversions
    pcl_ros
)

# Install the executable
install(TARGETS
    bbox_2d_3d_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()
