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
project(camera_trigger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(camera_trigger_msgs REQUIRED)

# ----------------------------
# Core library (no ROS): parsing, sequence tracking, pulse timing, UDP transport
# ----------------------------
add_library(camera_trigger_core STATIC
  src/protocol.cpp
  src/sequence_tracker.cpp
  src/pulse_timing.cpp
  src/transport.cpp
)
set_target_properties(camera_trigger_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(camera_trigger_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# ----------------------------
# ROS node, as a component plus its own executable (camera_trigger_node)
# ----------------------------
add_library(camera_trigger_component SHARED
  src/camera_trigger_node.cpp
)
target_link_libraries(camera_trigger_component
  camera_trigger_core
  rclcpp::rclcpp
  rclcpp_components::component
  ${builtin_interfaces_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${camera_trigger_msgs_TARGETS}
)

rclcpp_components_register_node(camera_trigger_component
  PLUGIN "camera_trigger::CameraTriggerNode"
  EXECUTABLE camera_trigger_node
)

# ----------------------------
# Install
# ----------------------------
install(TARGETS camera_trigger_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS tools/fake_stm32.py
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------
# Tests
# ----------------------------
if(BUILD_TESTING)
  find_package(wato_test REQUIRED)
  add_wato_test(test_protocol test/test_protocol.cpp LIBRARIES camera_trigger_core)
  add_wato_test(test_sequence_tracker test/test_sequence_tracker.cpp LIBRARIES camera_trigger_core)
  add_wato_test(test_pulse_timing test/test_pulse_timing.cpp LIBRARIES camera_trigger_core)
  add_wato_test(test_transport test/test_transport.cpp LIBRARIES camera_trigger_core)
  # The node end to end, against a fake board on loopback
  add_wato_test(test_camera_trigger_node test/test_camera_trigger_node.cpp LIBRARIES camera_trigger_component)
endif()

ament_package()
