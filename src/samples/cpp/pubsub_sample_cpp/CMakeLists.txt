cmake_minimum_required(VERSION 3.10)
project(pubsub_sample_cpp)

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
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs)

add_library(producer_lib src/producer.cpp)
target_include_directories(producer_lib
  PUBLIC include)
rosidl_target_interfaces(producer_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(producer_lib rclcpp std_msgs)

add_library(transformer_lib
  src/transformer.cpp)
target_include_directories(transformer_lib
  PUBLIC
  include
  isEven_cpp/src)
rosidl_target_interfaces(transformer_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(transformer_lib rclcpp std_msgs)

add_library(aggregator_lib src/aggregator.cpp)
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

  ament_add_gtest(producer_test test/producer_test.cpp)
  target_link_libraries(producer_test
    producer_lib)
  ament_add_gtest(transformer_test test/transformer_test.cpp)
  target_link_libraries(transformer_test
    transformer_lib
    gtest_main)
  ament_add_gtest(aggregator_test test/aggregator_test.cpp)
  target_link_libraries(aggregator_test
    aggregator_lib)

  install(TARGETS
    producer_test
    transformer_test
    DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(producer src/producer_node.cpp)
target_link_libraries(producer producer_lib)

add_executable(transformer src/transformer_node.cpp)
target_link_libraries(transformer transformer_lib)

add_executable(aggregator src/aggregator_node.cpp)
target_link_libraries(aggregator aggregator_lib)

install(TARGETS
  producer
  transformer
  aggregator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
