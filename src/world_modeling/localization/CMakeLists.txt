cmake_minimum_required(VERSION 3.10)
project(localization)

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
find_package(nav_msgs REQUIRED)

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(localization_lib
  src/localization_node.cpp
  src/odom.cpp
  src/odom_mock_data.cpp)
# Indicate to compiler where to search for header files
target_include_directories(localization_lib
  PUBLIC
  include)
# Add ROS2 dependencies required by package
ament_target_dependencies(localization_lib
  std_msgs
  nav_msgs
  rclcpp)

# Create ROS2 node executable from source files

#odom is the name of the node which we can run
add_executable(localization_node src/localization_node.cpp)
add_executable(odom src/odom.cpp)

# add_executable(odom_mock_data src/odom_mock_data.cpp)
# Link to the previously built library to access occupancy classes and methods
target_link_libraries(localization_node localization_lib)
target_link_libraries(odom localization_lib)

# target_link_libraries(odom_mock_data odom_lib)

# Copy executable to installation location
install(TARGETS
  localization
  odom
  # odom_mock_data
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
