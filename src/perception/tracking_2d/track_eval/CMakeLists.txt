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
project(track_eval)

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
find_package(vision_msgs REQUIRED)
find_package(bytetrack_cpp_vendor REQUIRED)

# Add the executable for the track_eval node
add_executable(track_eval_node
    src/track_eval.cpp
    src/Accumulator.cpp
    src/Matches.cpp
    src/cost.cpp
    src/lap.cpp
)

# Include directories
target_include_directories(track_eval_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link libraries
target_link_libraries(track_eval_node
    PRIVATE
    bytetrack_cpp_vendor::bytetrack
    rclcpp::rclcpp
    ${vision_msgs_TARGETS}
)

# Install the executable
install(TARGETS track_eval_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install launch files (if any)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_package()
