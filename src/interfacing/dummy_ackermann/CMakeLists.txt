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
cmake_minimum_required(VERSION 3.8)
project(dummy_ackermann)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Component Library
add_library(${PROJECT_NAME}_nodes SHARED
  src/ackermann_square_wave_node.cpp
  src/ackermann_velocity_trapezoid_node.cpp
)

target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_nodes PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  ackermann_msgs::ackermann_msgs__rosidl_typesupport_cpp
)

# Register Components
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "dummy_ackermann::AckermannSquareWaveNode"
  EXECUTABLE ackermann_square_wave_node
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "dummy_ackermann::AckermannVelocityTrapezoidNode"
  EXECUTABLE ackermann_velocity_trapezoid_node
)

# Install
install(TARGETS ${PROJECT_NAME}_nodes
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
