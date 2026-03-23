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
project(pid_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(roscco_msg REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pid_msgs REQUIRED)

add_library(pid_control_component SHARED
  src/pid_control_node.cpp
)

target_include_directories(pid_control_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to library
target_link_libraries(pid_control_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  ${lifecycle_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  roscco_msg::roscco_msg__rosidl_typesupport_cpp
  control_toolbox::control_toolbox
)

rclcpp_components_register_node(pid_control_component
  PLUGIN "pid_control::PidControlNode"
  EXECUTABLE pid_control_node
)

# Velocity-Driven Feedforward PID
add_library(vel_driven_feedforward_pid_component SHARED
  src/vel_driven_feedforward_pid_node.cpp
)

target_include_directories(vel_driven_feedforward_pid_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(vel_driven_feedforward_pid_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  ${lifecycle_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  roscco_msg::roscco_msg__rosidl_typesupport_cpp
  pid_msgs::pid_msgs__rosidl_typesupport_cpp
  control_toolbox::control_toolbox
)

rclcpp_components_register_node(vel_driven_feedforward_pid_component
  PLUGIN "pid_control::VelDrivenFeedforwardPidNode"
  EXECUTABLE vel_driven_feedforward_pid_node
)

# Ackermann Smoother (setpoint rate limiter)
add_library(ackermann_smoother_component SHARED
  src/ackermann_smoother_node.cpp
)

target_include_directories(ackermann_smoother_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ackermann_smoother_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  ${ackermann_msgs_TARGETS}
)

rclcpp_components_register_node(ackermann_smoother_component
  PLUGIN "pid_control::AckermannSmootherNode"
  EXECUTABLE ackermann_smoother_node
)

install(TARGETS
  pid_control_component
  vel_driven_feedforward_pid_component
  ackermann_smoother_component
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)
  add_wato_test(pid_control_node_test test/pid_control_node_test.cpp
    LIBRARIES pid_control_component
  )
endif()

ament_package()
