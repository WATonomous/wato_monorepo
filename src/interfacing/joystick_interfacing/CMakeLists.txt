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
project(joystick_interfacing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(roscco_msg REQUIRED)

# Create shared library component
add_library(joystick_interfacing SHARED
  src/joystick_interfacing.cpp
)

target_include_directories(joystick_interfacing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to library
target_link_libraries(joystick_interfacing PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  roscco_msg::roscco_msg__rosidl_typesupport_cpp
)

# Register component and automatically create standalone executable
# This replaces the need for manual add_executable and main() function
rclcpp_components_register_node(joystick_interfacing
  PLUGIN "joystick_node::JoystickNode"
  EXECUTABLE joystick_interfacing_node
)

# Install library and auto-generated executable
install(TARGETS joystick_interfacing
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)
  add_wato_test(joystick_interfacing_test test/joystick_interfacing_test.cpp
    LIBRARIES joystick_interfacing
  )

  add_wato_test(joystick_arming_test test/joystick_arming_test.cpp
    LIBRARIES joystick_interfacing
  )
endif()

ament_package()
