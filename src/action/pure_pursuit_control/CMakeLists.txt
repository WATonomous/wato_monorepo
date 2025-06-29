cmake_minimum_required(VERSION 3.10)
project(pure_pursuit_control)

# Set C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(world_modeling_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the library containing the PurePursuitController class
add_library(pure_pursuit_control_lib
  src/pure_pursuit_control.cpp
)

target_include_directories(pure_pursuit_control_lib
  PUBLIC
  include
)

ament_target_dependencies(
  pure_pursuit_control_lib
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  carla_msgs
  world_modeling_msgs
  tf2_geometry_msgs
  tf2
)

# Add the main node executable
add_executable(pure_pursuit_control src/pure_pursuit_control.cpp)

target_link_libraries(pure_pursuit_control
  pure_pursuit_control_lib
)

ament_target_dependencies(
  pure_pursuit_control
  rclcpp
)

# Install targets
install(TARGETS
  pure_pursuit_control
  pure_pursuit_control_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
