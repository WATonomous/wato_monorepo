cmake_minimum_required(VERSION 3.10)
project(sample_msgs)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED) # ROS2 build tool
find_package(std_msgs REQUIRED) # ROS2 common messages

set(msg_files
    msg/FilterArrayAverage.msg
    msg/Filtered.msg
    msg/FilteredArray.msg
    msg/Unfiltered.msg
    msg/Metadata.msg)
# Parses message files to generate ROS2 interfaces
rosidl_generate_interfaces(sample_msgs
    ${msg_files}
    DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
