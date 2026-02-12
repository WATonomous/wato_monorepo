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
project(car_velocity_feedback)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roscco_msg REQUIRED)

# add include directories
include_directories(include)

# create library
add_library(car_velocity_feedback_node_lib src/car_velocity_feedback_node.cpp)

target_include_directories(car_velocity_feedback_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(car_velocity_feedback_node_lib PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  roscco_msg::roscco_msg__rosidl_typesupport_cpp
)

# register component
rclcpp_components_register_node(car_velocity_feedback_node_lib
  PLUGIN "car_velocity_feedback::CarVelocityFeedbackNode"
  EXECUTABLE car_velocity_feedback_node
)

# install
install(TARGETS car_velocity_feedback_node_lib car_velocity_feedback_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # specific tests
  find_package(wato_test REQUIRED)
  add_wato_test(${PROJECT_NAME}_test test/car_velocity_feedback_test.cpp
    LIBRARIES car_velocity_feedback_node_lib
  )
endif()

ament_package()
