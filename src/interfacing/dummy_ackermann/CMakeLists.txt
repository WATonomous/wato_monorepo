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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Ackermann Square Wave Node
add_library(ackermann_square_wave_lib SHARED
  src/ackermann_square_wave_node.cpp
)
target_include_directories(ackermann_square_wave_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ackermann_square_wave_lib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  ackermann_msgs
)
rclcpp_components_register_node(ackermann_square_wave_lib
  PLUGIN "dummy_ackermann::AckermannSquareWaveNode"
  EXECUTABLE ackermann_square_wave_node
)

# Ackermann Velocity Trapezoid Node
add_library(ackermann_velocity_trapezoid_lib SHARED
  src/ackermann_velocity_trapezoid_node.cpp
)
target_include_directories(ackermann_velocity_trapezoid_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ackermann_velocity_trapezoid_lib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  ackermann_msgs
)
rclcpp_components_register_node(ackermann_velocity_trapezoid_lib
  PLUGIN "dummy_ackermann::AckermannVelocityTrapezoidNode"
  EXECUTABLE ackermann_velocity_trapezoid_node
)

install(TARGETS
  ackermann_square_wave_lib
  ackermann_velocity_trapezoid_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
