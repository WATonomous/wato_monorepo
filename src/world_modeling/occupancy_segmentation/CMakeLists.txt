cmake_minimum_required(VERSION 3.10)
project(occupancy_segmentation)

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
find_package(sample_msgs REQUIRED) # Custom package containing ROS2 messages
find_package(pcl_ros REQUIRED)

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(occupancy_segmentation_lib
  src/occupancy_segmentation_core.cpp)
# Indicate to compiler where to search for header files
target_include_directories(occupancy_segmentation_lib
  PUBLIC
  include)
# Add ROS2 dependencies required by package
ament_target_dependencies(occupancy_segmentation_lib
  rclcpp
  sample_msgs
  pcl_ros)

# # By default tests are built. This can be overridden by modifying the
# # colcon build command to include the -DBUILD_TESTING=OFF flag.
# option(BUILD_TESTING "Build tests" ON)
# if(BUILD_TESTING)
#   # Search for dependencies required for building tests + linting
#   find_package(ament_cmake_gtest REQUIRED)

#   # Create test executable from source files
#   ament_add_gtest(occupancy_segmentation_test test/occupancy_segmentation_test.cpp)
#   # Link to the previously built library to access occupancy_segmentation classes and methods
#   target_link_libraries(occupancy_segmentation_test
#     occupancy_segmentation_lib
#     gtest_main)

#   # Copy executable to installation location
#   install(TARGETS
#     occupancy_segmentation_test
#     DESTINATION lib/${PROJECT_NAME})
# endif()

# Create ROS2 node executable from source files
add_executable(occupancy_segmentation_node src/occupancy_segmentation_node.cpp)
# Link to the previously built library to access occupancy_segmentation classes and methods
target_link_libraries(occupancy_segmentation_node occupancy_segmentation_lib)

# Copy executable to installation location
install(TARGETS
  occupancy_segmentation_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

