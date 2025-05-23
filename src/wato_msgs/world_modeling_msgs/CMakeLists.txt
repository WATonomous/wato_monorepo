cmake_minimum_required(VERSION 3.10)
project(world_modeling_msgs)

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
find_package(geometry_msgs REQUIRED) # ROS2 common messages
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/VoxelGrid.msg
  msg/LaneletPath.msg
  msg/Lanelet.msg
  )

set(srv_files
  srv/LaneletInfo.srv
  srv/BehaviourTreeInfo.srv
  )

# Parses message files to generate ROS2 interfaces
rosidl_generate_interfaces(world_modeling_msgs
    ${msg_files}
    ${srv_files}
    DEPENDENCIES std_msgs geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
