cmake_minimum_required(VERSION 3.5)
project(perception_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

# Include header files
include_directories(include
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(perception_utils SHARED
  src/camera_utils.cpp
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(perception_utils
  rclcpp
  OpenCV)

install(TARGETS perception_utils
  EXPORT export_perception_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_perception_utils HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()