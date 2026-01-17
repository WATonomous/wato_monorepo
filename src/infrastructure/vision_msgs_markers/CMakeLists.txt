cmake_minimum_required(VERSION 3.10)
project(vision_msgs_markers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vision_msgs_markers_node src/vision_msgs_markers_node.cpp)

target_include_directories(vision_msgs_markers_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(vision_msgs_markers_node
  PRIVATE
  rclcpp::rclcpp
  ${vision_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS vision_msgs_markers_node RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
