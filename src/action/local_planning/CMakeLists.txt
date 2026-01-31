cmake_minimum_required(VERSION 3.8)
project(local_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(local_planner_core SHARED
  src/local_planner_core.cpp
)
target_include_directories(local_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Put only what local_planner_core uses:
ament_target_dependencies(local_planner_core)

add_executable(local_planner
  src/local_planner_node.cpp src/local_planner_core.cpp
)
target_include_directories(local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(local_planner_core PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(local_planner local_planner_core Eigen3::Eigen)
ament_target_dependencies(local_planner
  rclcpp rclcpp_lifecycle rclcpp_components
  std_msgs lanelet_msgs nav_msgs geometry_msgs
  visualization_msgs tf2 tf2_geometry_msgs
)

install(TARGETS local_planner local_planner_core
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
