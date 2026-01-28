cmake_minimum_required(VERSION 3.8)
project(mppi_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(mppi_core src/mppi_core.cpp)

target_include_directories(mppi_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

#giving ros2 dependencies to mppi_core, remove if not needed
ament_target_dependencies(mppi_core
  rclcpp
  nav_msgs
  trajectory_msgs
  ackermann_msgs
  tf2_msgs
)

add_executable(mppi_node src/mppi_node.cpp)

target_link_libraries(mppi_node mppi_core)
target_include_directories(mppi_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Let ament set up include dirs and link flags for rclcpp
ament_target_dependencies(mppi_node
  rclcpp
  nav_msgs
  trajectory_msgs
  ackermann_msgs
  tf2_msgs
)

install(
  TARGETS mppi_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()