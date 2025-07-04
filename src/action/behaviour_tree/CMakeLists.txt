cmake_minimum_required(VERSION 3.10)
project(behaviour_tree)

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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(world_modeling_msgs REQUIRED)

# Add the library containing the BehaviourTree class
add_library(behaviour_tree_lib
  src/behaviour_tree.cpp
)

target_include_directories(behaviour_tree_lib
  PUBLIC
  include
)

ament_target_dependencies(
  behaviour_tree_lib
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  world_modeling_msgs
)

# Add the main node executable
add_executable(behaviour_tree src/behaviour_tree.cpp)

target_link_libraries(behaviour_tree
  behaviour_tree_lib
)

ament_target_dependencies(
  behaviour_tree
  rclcpp
)

# Install targets
install(TARGETS
  behaviour_tree
  behaviour_tree_lib
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
