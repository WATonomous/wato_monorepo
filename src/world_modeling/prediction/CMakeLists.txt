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
project(prediction)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- General ROS 2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# ---- Msg Packages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(prediction_lib
  src/trajectory_predictor.cpp
  src/intent_classifier.cpp
  src/motion_models.cpp
  src/map_interface.cpp
)

set_target_properties(prediction_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(prediction_lib
  PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR})

# Link ROS2 dependencies required by package
target_link_libraries(prediction_lib
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  Eigen3::Eigen
  )

add_library(prediction_node SHARED src/prediction_node.cpp)
target_link_libraries(prediction_node prediction_lib)
rclcpp_components_register_nodes(prediction_node "prediction::PredictionNode")

install(TARGETS
  prediction_lib
  prediction_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
