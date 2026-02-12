cmake_minimum_required(VERSION 3.8)
project(wato_trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(wato_trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(wato_lifecycle_manager REQUIRED)

include_directories(include)

# Component Library
add_library(${PROJECT_NAME}_component SHARED
  src/trajectory_core.cpp
  src/trajectory_planner_node.cpp
)

target_include_directories(${PROJECT_NAME}_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  nav_msgs
  geometry_msgs
  visualization_msgs
  lanelet_msgs
  wato_trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  wato_lifecycle_manager
)

# Register component and generate executable
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "wato_trajectory_planner::TrajectoryPlannerNode"
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

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
