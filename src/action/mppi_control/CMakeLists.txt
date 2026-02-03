cmake_minimum_required(VERSION 3.8)
project(mppi_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathWithSpeed.msg"
  DEPENDENCIES std_msgs nav_msgs
)

# Build as a SHARED library so it produces libmppi_core.so
add_library(mppi_core SHARED src/mppi_core.cpp src/critics.cpp)

target_include_directories(mppi_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mppi_core
  rclcpp
  nav_msgs
  std_msgs
  trajectory_msgs
  ackermann_msgs
  tf2_msgs
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
  trajectory_msgs
  ackermann_msgs
  tf2_msgs
  std_msgs
)

# ---- Non-deprecated ROSIDL typesupport linking (Jazzy+) ----
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mppi_core "${cpp_typesupport_target}")
target_link_libraries(mppi_node "${cpp_typesupport_target}")
# ------------------------------------------------------------

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

# Export the library so other packages can link against it (optional but correct)
ament_export_libraries(mppi_core)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
