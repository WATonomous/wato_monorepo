cmake_minimum_required(VERSION 3.10)
project(wato_test_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(autoware_perception_msgs REQUIRED)

set(msg_files
    msg/TrafficLightElement.msg
    msg/TrafficLightGroup.msg
    msg/TrafficLightGroupArray.msg)

rosidl_generate_interfaces(wato_test_msgs
    ${msg_files}
    DEPENDENCIES std_msgs
    DEPENDENCIES builtin_interfaces
    DEPENDENCIES autoware_perception_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()