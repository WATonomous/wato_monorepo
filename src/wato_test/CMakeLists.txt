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

cmake_minimum_required(VERSION 3.22)
project(wato_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options(-Wl,-no-undefined)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
set(include_dir ${CMAKE_CURRENT_SOURCE_DIR}/include)

# wato_test library
set(WATO_TEST_LIB ${PROJECT_NAME}_lib)
add_library(${WATO_TEST_LIB} SHARED
  src/test_executor_fixture.cpp
)
target_include_directories(${WATO_TEST_LIB} PUBLIC
  $<BUILD_INTERFACE:${include_dir}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${WATO_TEST_LIB}
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
)

install(TARGETS
  ${WATO_TEST_LIB}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES cmake/add_wato_test.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  # Include our custom test function
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/add_wato_test.cmake)

  # Find test dependencies
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  # Add the basic example test
  add_wato_test(test_basic_example
    test/test_basic_example.cpp
    LIBRARIES
      ${std_msgs_TARGETS}
      ${std_srvs_TARGETS}
  )

  # Add ROS test with multiple library dependencies
  add_wato_test(test_with_ros
    test/test_with_ros.cpp
    LIBRARIES
      ${std_msgs_TARGETS}
  )
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)
