cmake_minimum_required(VERSION 3.10)
project(occupancy)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED) # ROS2 python build tool
find_package(ament_cmake_python REQUIRED) # ROS2 python build tool
find_package(rclcpp REQUIRED)      # ROS2 C++ package
find_package(rclpy REQUIRED)       # ROS2 python package
find_package(sample_msgs REQUIRED) # Custom package containing ROS2 messages
find_package(world_modeling_msgs REQUIRED) # Custom package containing ROS2 messages
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(sample_msgs)
ament_export_dependencies(world_modeling_msgs)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(pcl_ros)


# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(sample_lib
  src/sample.cpp)
# Indicate to compiler where to search for header files
target_include_directories(sample_lib
  PUBLIC
  include)
# Add ROS2 dependencies required by package
ament_target_dependencies(sample_lib
  rclcpp
  sample_msgs
  world_modeling_msgs)

# Create ROS2 node executable from source files
add_executable(sample_node src/sample_node.cpp)
add_executable(voxelizer_node src/voxelizer_node.cpp)

# Link to the previously built library to access Sample classes and methods
target_link_libraries(sample_node sample_lib)

ament_target_dependencies(voxelizer_node
  rclcpp
  sample_msgs
  world_modeling_msgs
  pcl_conversions
  pcl_ros)

target_include_directories(voxelizer_node
  PUBLIC
  include)

  # Copy executable to installation location
install(TARGETS
  sample_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  voxelizer_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  py/voxel_segmentation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
