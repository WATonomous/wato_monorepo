cmake_minimum_required(VERSION 3.8)
project(car_steering_feedback)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roscco_msg REQUIRED)

add_executable(car_steering_feedback_node
  src/car_steering_feedback_node.cpp
)

target_include_directories(car_steering_feedback_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(car_steering_feedback_node
  rclcpp::rclcpp
  roscco_msg::roscco_msg__rosidl_typesupport_cpp
)

install(TARGETS car_steering_feedback_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
