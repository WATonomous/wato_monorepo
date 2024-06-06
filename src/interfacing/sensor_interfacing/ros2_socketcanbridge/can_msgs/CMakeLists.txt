cmake_minimum_required(VERSION 3.10)

project(can_msgs)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake               REQUIRED)
find_package(builtin_interfaces        REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs             REQUIRED)
find_package(std_msgs                  REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Frame.msg"
  DEPENDENCIES builtin_interfaces  geometry_msgs std_msgs
)

ament_package()
