cmake_minimum_required(VERSION 3.14)
project(local_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(local_planning_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(behaviour_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ----------------------------
# Component library
# ----------------------------
add_library(local_planning_component SHARED
  src/local_planner_core.cpp
  src/local_planner_node.cpp
)

target_include_directories(local_planning_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(local_planning_component
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${rclcpp_components_TARGETS}
    ${std_msgs_TARGETS}
    ${lanelet_msgs_TARGETS}
    ${local_planning_msgs_TARGETS}
    ${behaviour_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    Eigen3::Eigen
)

# Register component and generate executable
rclcpp_components_register_node(
  local_planning_component
  PLUGIN "LocalPlannerNode"
  EXECUTABLE local_planner_node
)

# ----------------------------
# Install
# ----------------------------
install(
  TARGETS local_planning_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()