cmake_minimum_required(VERSION 3.10)
project(lane_detection_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
    msg/LaneLine.msg
    msg/LaneDetection.msg)

rosidl_generate_interfaces(lane_detection_msgs
    ${msg_files}
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()