cmake_minimum_required(VERSION 3.10)

project(ros2socketcan_bridge)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)

include_directories(include)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp can_msgs)

add_library(ros2socketcan_bridge STATIC
        include/ros2socketcan.h)

set_target_properties(ros2socketcan_bridge PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(ros2socketcan_bridge)

ament_target_dependencies(ros2socketcan_bridge rclcpp can_msgs)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME})

install(TARGETS ros2socketcan_bridge
        DESTINATION lib/${PROJECT_NAME})

ament_package()
