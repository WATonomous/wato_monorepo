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
project(ackermann_pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(wato_trajectory_msgs REQUIRED)

add_library(pure_pursuit_lib SHARED
  src/pure_pursuit_node.cpp
)

target_include_directories(pure_pursuit_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(pure_pursuit_lib
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${ackermann_msgs_TARGETS}
    ${wato_trajectory_msgs_TARGETS}
  PRIVATE
    tf2_geometry_msgs::tf2_geometry_msgs
)

add_executable(pure_pursuit_exe src/main.cpp)
set_target_properties(pure_pursuit_exe PROPERTIES OUTPUT_NAME pure_pursuit_node)

target_link_libraries(pure_pursuit_exe
  PRIVATE
    pure_pursuit_lib
)

install(TARGETS pure_pursuit_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pure_pursuit_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
