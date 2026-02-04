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
project(carla_lifecycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Library
add_library(${PROJECT_NAME}_lib SHARED
  src/lifecycle_manager_node.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lib
  rclcpp::rclcpp
  ${lifecycle_msgs_TARGETS}
  ${carla_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

# Executable
add_executable(lifecycle_manager src/main.cpp)
target_link_libraries(lifecycle_manager
  ${PROJECT_NAME}_lib
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS lifecycle_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)

  add_wato_test(test_lifecycle_manager_node
    test/test_lifecycle_manager_node.cpp
    LIBRARIES
      ${PROJECT_NAME}_lib
  )
endif()

ament_package()
