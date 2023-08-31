cmake_minimum_required(VERSION 3.10)
project(sample_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
    msg/Filtered.msg
    msg/FilteredArray.msg
    msg/Unfiltered.msg
    msg/Metadata.msg)
rosidl_generate_interfaces(sample_msgs
    ${msg_files}
    DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()