cmake_minimum_required(VERSION 3.10)
project(simple_prediction)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(world_model_msgs REQUIRED)

add_executable(simple_prediction_node src/simple_prediction_node.cpp)

target_include_directories(simple_prediction_node PRIVATE include)

target_link_libraries(simple_prediction_node
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${world_model_msgs_TARGETS}
)

install(TARGETS simple_prediction_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
