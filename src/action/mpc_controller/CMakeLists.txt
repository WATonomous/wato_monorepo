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
cmake_minimum_required(VERSION 3.14)
project(mpc_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(osqp_eigen_vendor REQUIRED)  # sets up CMAKE_PREFIX_PATH for OsqpEigen
find_package(OsqpEigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviour_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wato_trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(wato_lifecycle_manager REQUIRED)

# Component Library
add_library(${PROJECT_NAME}_component SHARED
  src/bicycle_model.cpp
  src/mpc_core.cpp
  src/mpc_controller_node.cpp
)

target_include_directories(${PROJECT_NAME}_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_component
  PUBLIC
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
    ${wato_trajectory_msgs_TARGETS}
    ${nav_msgs_TARGETS}
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${rclcpp_components_TARGETS}
    ${geometry_msgs_TARGETS}
    ${behaviour_msgs_TARGETS}
    ${ackermann_msgs_TARGETS}
    ${std_msgs_TARGETS}
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${wato_lifecycle_manager_TARGETS}
)

# Register component and generate executable
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "mpc_controller::MpcControllerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS
  ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TARGET ${PROJECT_NAME}_node)
  install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)

  add_wato_test(test_bicycle_model
    test/test_bicycle_model.cpp
    LIBRARIES
      ${PROJECT_NAME}_component
  )

  add_wato_test(test_mpc_core
    test/test_mpc_core.cpp
    LIBRARIES
      ${PROJECT_NAME}_component
  )
endif()

ament_package()
