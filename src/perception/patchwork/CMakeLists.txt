cmake_minimum_required(VERSION 3.16...3.26)
project(patchworkpp VERSION 1.0.4 LANGUAGES CXX)

set(ignore ${CATKIN_INSTALL_INTO_PREFIX_ROOT})
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ROS 2 dependencies first
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# PCL and Eigen3 dependencies
find_package(PCL REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Try to find Patchwork++ using find_package first
# The package should be installed to /usr/local by the Dockerfile
find_package(patchworkpp QUIET HINTS /usr/local/lib/cmake/patchworkpp)

if(TARGET patchworkpp::ground_seg_cores)
  # Use the proper CMake target if available
  message(STATUS "Found Patchwork++ via find_package")
  set(PATCHWORKPP_TARGET patchworkpp::ground_seg_cores)
else()
  # Fallback to manual library finding
  find_path(PATCHWORKPP_INCLUDE_DIR
    NAMES patchwork/patchworkpp.h
    HINTS /usr/local/include /usr/include
    PATH_SUFFIXES patchworkpp
  )
  find_library(PATCHWORKPP_LIBRARY
    NAMES ground_seg_cores
    HINTS /usr/local/lib /usr/lib /usr/local/lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu
  )

  if(NOT PATCHWORKPP_INCLUDE_DIR OR NOT PATCHWORKPP_LIBRARY)
    message(STATUS "PATCHWORKPP_INCLUDE_DIR: ${PATCHWORKPP_INCLUDE_DIR}")
    message(STATUS "PATCHWORKPP_LIBRARY: ${PATCHWORKPP_LIBRARY}")
    message(FATAL_ERROR "Patchwork++ not found. Ensure it is installed in the image.")
  endif()

  message(STATUS "Found Patchwork++ include dir: ${PATCHWORKPP_INCLUDE_DIR}")
  message(STATUS "Found Patchwork++ library: ${PATCHWORKPP_LIBRARY}")

  # Create interface target for manual linking
  add_library(patchworkpp_vendor INTERFACE)
  target_include_directories(patchworkpp_vendor INTERFACE ${PATCHWORKPP_INCLUDE_DIR})
  target_link_libraries(patchworkpp_vendor INTERFACE ${PATCHWORKPP_LIBRARY} ${PCL_LIBRARIES})
  set(PATCHWORKPP_TARGET patchworkpp_vendor)
endif()

# ROS 2 ground removal node
add_library(ground_removal_component SHARED src/ground_removal.cpp)
target_compile_features(ground_removal_component PUBLIC cxx_std_20)
target_include_directories(ground_removal_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(ground_removal_component PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(ground_removal_component ${PATCHWORKPP_TARGET})
ament_target_dependencies(
  ground_removal_component
  rcutils
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  eigen3_cmake_module)

rclcpp_components_register_node(ground_removal_component PLUGIN "patchworkpp_ros::GroundRemovalServer" EXECUTABLE patchworkpp_ground_removal_node)

install(TARGETS ground_removal_component LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(Eigen3)

ament_package()
