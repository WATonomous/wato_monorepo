cmake_minimum_required(VERSION 3.10)
project(continental_driver)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED) # ROS2 build tool
find_package(rclcpp REQUIRED)      # ROS2 C++ package
find_package(continental_msgs REQUIRED) # Custom package containing ROS2 messages

add_library(continental_lib
  src/driver.cpp)
target_include_directories(continental_lib
  PUBLIC include)
ament_target_dependencies(continental_lib rclcpp continental_msgs)

# By default tests are built. This can be overridden by modifying the
# colcon build command to include the -DBUILD_TESTING=OFF flag.
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  # Search for dependencies required for building tests + linting
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  # Remove the default C++ and copyright linter
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_copyright)

  # Reinstall cpplint ignoring copyright errors
  ament_cpplint(FILTERS "-legal/copyright")

  ament_lint_auto_find_test_dependencies()
endif()

add_executable(continental_node src/continental_node.cpp)
target_link_libraries(continental_node continental_lib)

install(TARGETS
  continental_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
