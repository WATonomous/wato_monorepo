cmake_minimum_required(VERSION 3.10)
project(radar_vis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED) # ROS2 package containing point cloud messages
find_package(radar_msgs REQUIRED)  # Custom package containing ROS2 Radar messages

add_library(radar_vis_lib
  src/radar_vis.cpp)

target_include_directories(radar_vis_lib
  PUBLIC include)

ament_target_dependencies(radar_vis_lib rclcpp radar_msgs sensor_msgs)

option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_copyright)

  ament_cpplint(FILTERS "-legal/copyright")

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(radar_vis_test test/radar_vis_test.cpp)

  target_link_libraries(radar_vis_test radar_vis_lib)

  install(TARGETS
    radar_vis_test
    DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(radar_vis_node src/radar_vis_node.cpp)

target_link_libraries(radar_vis_node radar_vis_lib)

install(TARGETS
  radar_vis_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
