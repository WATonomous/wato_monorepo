cmake_minimum_required(VERSION 3.10)
project(odom_test)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED) # ROS2 build tool
find_package(rclcpp REQUIRED)      # ROS2 C++ package
find_package(std_msgs REQUIRED)      # ROS2 Messages

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(odom_lib
  src/odom.cpp)
# Indicate to compiler where to search for header files
target_include_directories(odom_lib
  PUBLIC
  include)
# Add ROS2 dependencies required by package
ament_target_dependencies(odom_lib
  std_msgs
  rclcpp)

# Create ROS2 node executable from source files

#odom_node is the name of the node which we can run
add_executable(odom_node src/odom.cpp)
# Link to the previously built library to access occupancy classes and methods
target_link_libraries(odom_node odom_lib)

# Copy executable to installation location
install(TARGETS
  odom_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

