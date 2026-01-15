cmake_minimum_required(VERSION 3.8)
project(carla_lifecycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(carla_msgs REQUIRED)

add_executable(lifecycle_manager
  src/lifecycle_manager_node.cpp
  src/main.cpp
)
target_include_directories(lifecycle_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lifecycle_manager
  rclcpp::rclcpp
  ${lifecycle_msgs_TARGETS}
  ${carla_msgs_TARGETS}
)

install(TARGETS lifecycle_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
