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
project(wato_patchworkpp)

# C++ standard & warnings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 & deps
find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Patchwork++ (it exports different target names across installs, so normalize it to PATCHWORKPP_TARGET
find_package(patchworkpp REQUIRED)
if(TARGET patchworkpp::ground_seg_cores)
  set(PATCHWORKPP_TARGET patchworkpp::ground_seg_cores)
elseif(TARGET patchworkpp::patchworkpp)
  set(PATCHWORKPP_TARGET patchworkpp::patchworkpp)
else()
  message(FATAL_ERROR "Found patchworkpp but no expected exported target")
endif()

add_executable(patchworkpp_node
  src/ground_removal_core.cpp
  src/ground_removal_node.cpp
)

target_include_directories(patchworkpp_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Use the PLAIN signature here (no PRIVATE/PUBLIC/INTERFACE) to match ament_target_dependencies.
target_link_libraries(patchworkpp_node
  PRIVATE
    ${PATCHWORKPP_TARGET}
    ${sensor_msgs_TARGETS}
    ${geometry_msg_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
  PUBLIC
    Eigen3::Eigen
    rcutils::rcutils
    rclcpp::rclcpp
    tf2_ros::tf2_ros
)

install(TARGETS patchworkpp_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
