cmake_minimum_required(VERSION 3.10)
project(patchworkpp)

# C++ standard & warnings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 & deps
find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Patchwork++ (CMake package)

find_package(patchworkpp REQUIRED)
if(TARGET patchworkpp::ground_seg_cores)
  set(PATCHWORKPP_TARGET patchworkpp::ground_seg_cores)
elseif(TARGET patchworkpp::patchworkpp)
  set(PATCHWORKPP_TARGET patchworkpp::patchworkpp)
else()
  message(FATAL_ERROR "Found patchworkpp but no expected exported target")
endif()


add_executable(patchworkpp_node
  src/ground_removal_core.cpp
  src/ground_removal_node.cpp
)

target_include_directories(patchworkpp_node
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(patchworkpp_node ${PATCHWORKPP_TARGET})

ament_target_dependencies(patchworkpp_node
  rcutils
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  std_msgs
  Eigen3
)

install(TARGETS patchworkpp_node RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Export
ament_export_dependencies(Eigen3)
ament_package()
