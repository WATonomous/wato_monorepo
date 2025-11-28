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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedPrediction.msg"
  "msg/TrackedPredictionArray.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(prediction_lib
  src/utils.cpp
  src/kalman_filter.cpp
)

# Indicate to compiler where to search for header files
target_include_directories(prediction_lib
  PUBLIC
  include)

# Add ROS2 dependencies required by package
ament_target_dependencies(prediction_lib
  rclcpp
  visualization_msgs
  geometry_msgs
  Eigen3
  rosidl_default_runtime
)

target_link_libraries(prediction_lib
  Eigen3::Eigen)

# # By default tests are built. This can be overridden by modifying the
# # colcon build command to include the -DBUILD_TESTING=OFF flag.
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  # Search for dependencies required for building tests + linting
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  find_package(catch_ros2 REQUIRED)
  find_package(wato_test REQUIRED)

  # Remove the default C++ and copyright linter
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_copyright)

  # Reinstall cpplint ignoring copyright errors
  ament_cpplint(FILTERS "-legal/copyright")

  ament_lint_auto_find_test_dependencies()

  # Integration test using catch2
  add_executable(test_kalman_filter_integration 
    test/test_kalman_filter_integration.cpp)
  
  target_link_libraries(test_kalman_filter_integration
    prediction_lib
    catch_ros2::catch_ros2
  )
  
  ament_target_dependencies(test_kalman_filter_integration
    rclcpp
    visualization_msgs
    geometry_msgs
    wato_test
    Eigen3
    rosidl_default_runtime
  )

  add_test(NAME test_kalman_filter_integration 
    COMMAND test_kalman_filter_integration)
endif()

# Create ROS2 node executable from source files
add_executable(prediction_node src/kalman_filter.cpp)
# Link to the previously built library to access Sample classes and methods
target_link_libraries(prediction_node prediction_lib)
ament_target_dependencies(prediction_node rclcpp rosidl_default_runtime)

# Copy executable to installation location
install(TARGETS
  prediction_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  msg
  DESTINATION share/${PROJECT_NAME})

ament_package()
