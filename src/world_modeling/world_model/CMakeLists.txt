cmake_minimum_required(VERSION 3.14)
project(world_model)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ══════════════════════════════════════════════════════════════════════
# Dependencies
# ══════════════════════════════════════════════════════════════════════

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(prediction_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)

# ══════════════════════════════════════════════════════════════════════
# Library
# ══════════════════════════════════════════════════════════════════════

add_library(${PROJECT_NAME}_lib SHARED
  src/lanelet_handler.cpp
  src/world_model_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  vision_msgs
  lanelet_msgs
  prediction_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
)

# ══════════════════════════════════════════════════════════════════════
# Executable
# ══════════════════════════════════════════════════════════════════════

add_executable(world_model_node
  src/world_model_node.cpp
  src/lanelet_handler.cpp
)

target_include_directories(world_model_node PRIVATE include)

ament_target_dependencies(world_model_node
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  vision_msgs
  lanelet_msgs
  prediction_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
)

# ══════════════════════════════════════════════════════════════════════
# Install
# ══════════════════════════════════════════════════════════════════════

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS world_model_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ══════════════════════════════════════════════════════════════════════
# Testing
# ══════════════════════════════════════════════════════════════════════

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
