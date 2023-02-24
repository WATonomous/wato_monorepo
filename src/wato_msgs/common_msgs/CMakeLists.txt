cmake_minimum_required(VERSION 3.10)
project(common_msgs)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED) # ROS2 common messages
find_package(geometry_msgs REQUIRED)


# Parses message files to generate ROS2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleList.msg"
  DEPENDENCIES std_msgs geometry_msgs)
  

ament_export_dependencies(rosidl_default_runtime)
ament_package()


