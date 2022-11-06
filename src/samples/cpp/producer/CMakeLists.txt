cmake_minimum_required(VERSION 3.10)
project(producer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sample_msgs REQUIRED)

add_library(producer_lib
  src/producer.cpp)
target_include_directories(producer_lib
  PUBLIC include)
ament_target_dependencies(producer_lib rclcpp sample_msgs)

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

  ament_add_gtest(producer_test test/producer_test.cpp)
  target_link_libraries(producer_test producer_lib)

  install(TARGETS
    producer_test
    DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(producer_node src/producer_node.cpp)
target_link_libraries(producer_node producer_lib)

install(TARGETS
  producer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
