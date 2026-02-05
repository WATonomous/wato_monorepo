cmake_minimum_required(VERSION 3.8)
project(mppi_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wato_trajectory_msgs REQUIRED)

# Build as a SHARED library so it produces libmppi_core.so
add_library(mppi_core SHARED src/mppi_core.cpp src/critics.cpp)

target_include_directories(mppi_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mppi_core
  rclcpp
  nav_msgs
  ackermann_msgs
  tf2_msgs
  wato_trajectory_msgs
)

add_executable(mppi_node src/mppi_node.cpp)
target_link_libraries(mppi_node mppi_core)

target_include_directories(mppi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mppi_node
  rclcpp
  nav_msgs
  ackermann_msgs
  tf2_msgs
  wato_trajectory_msgs
)

# Install BOTH the executable and the library
install(TARGETS
  mppi_node
  mppi_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers if you have include/
install(DIRECTORY include/ DESTINATION include)

# Install config files (so params YAML exists in install space)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export the library so other packages can link against it
ament_export_libraries(mppi_core)
ament_export_include_directories(include)

ament_package()
