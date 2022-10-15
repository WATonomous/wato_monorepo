cmake_minimum_required(VERSION 3.10)
project(aggregator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    msg/Filtered.msg
    msg/FilteredArray.msg
    msg/Unfiltered.msg
    msg/Metadata.msg)
rosidl_generate_interfaces(aggregator
    ${msg_files}
    DEPENDENCIES std_msgs)

add_library(aggregator_lib
  src/aggregator.cpp)
target_include_directories(aggregator_lib
  PUBLIC include)
rosidl_target_interfaces(aggregator_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(aggregator_lib rclcpp std_msgs)

option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
    ament_cmake_copyright)

  set(LINTER_DIRECTORY_INCLUDES
    include
    src
    test)
  ament_cpplint(${LINTER_DIRECTORY_INCLUDES}
    FILTERS "-legal/copyright")
  ament_uncrustify(${LINTER_DIRECTORY_INCLUDES})

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(aggregator_test test/aggregator_test.cpp)
  target_link_libraries(aggregator_test aggregator_lib)

  install(TARGETS
    aggregator_test
    DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(aggregator_node src/aggregator_node.cpp)
target_link_libraries(aggregator_node aggregator_lib)

install(TARGETS
  aggregator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
