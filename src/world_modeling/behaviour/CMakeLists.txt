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
project(behaviour)

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
find_package(std_msgs REQUIRED)    # ROS2 Messages
find_package(std_srvs REQUIRED)    # ROS2 Services
find_package(rclcpp_action REQUIRED) # ROS2 Action Library
find_package(behaviortree_cpp_v3 REQUIRED) # Core BehaviorTree.CPP library
find_package(behaviortree_ros2 REQUIRED) # ROS2 BehaviorTree Library
find_package(btcpp_ros2_interfaces REQUIRED) # ROS2 BehaviorTree Messages

# Compiles source files into a library
add_library(behaviour_lib
  src/behaviour_core.cpp
  src/bt_node_utils.cpp
  src/bt_nodes.cpp
)
# Indicate to compiler where to search for header files
target_include_directories(behaviour_lib
  PUBLIC
  include)

ament_target_dependencies(behaviour_lib
  rclcpp
  std_msgs
  std_srvs
  rclcpp_action
  behaviortree_cpp_v3
  behaviortree_ros2
  btcpp_ros2_interfaces
)

# Create ROS2 node executable from source files
add_executable(behaviour_node src/behaviour_node.cpp)

# Link to the previously built library to access behaviour classes and methods
target_link_libraries(behaviour_node behaviour_lib)
ament_target_dependencies(behaviour_node
  rclcpp
  std_msgs
  std_srvs
  rclcpp_action
  behaviortree_cpp_v3
  behaviortree_ros2
  btcpp_ros2_interfaces
)

# Copy executable to installation location
install(TARGETS
  behaviour_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
