cmake_minimum_required(VERSION 3.10)
project(test_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# add_library(transformer_lib
#   src/transformer.cpp)
# target_include_directories(transformer_lib
#   PUBLIC
#   include)
# ament_target_dependencies(transformer_lib
#   rclcpp
#   sample_msgs)

option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_copyright)

  ament_cpplint(FILTERS "-legal/copyright")

  ament_lint_auto_find_test_dependencies()
endif()

# add_executable(transformer_node src/transformer_node.cpp)
# target_link_libraries(transformer_node transformer_lib)

# install(TARGETS
#   transformer_node
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
