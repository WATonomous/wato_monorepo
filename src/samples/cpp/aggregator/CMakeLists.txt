cmake_minimum_required(VERSION 3.10)
project(aggregator)

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
find_package(sample_msgs REQUIRED) # Custom package containing ROS2 messages

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(aggregator_lib
  src/aggregator_core.cpp)
# Indicate to compiler where to search for header files
target_include_directories(aggregator_lib
  PUBLIC include)
# Add ROS2 dependencies required by package
ament_target_dependencies(aggregator_lib rclcpp sample_msgs)

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

  # Create test executable from source files
  ament_add_gtest(aggregator_test test/aggregator_test.cpp)
  # Link to the previously built library to access Aggregator classes and methods
  target_link_libraries(aggregator_test aggregator_lib)

  # Copy executable to installation location
  install(TARGETS
    aggregator_test
    DESTINATION lib/${PROJECT_NAME})
endif()

# Create ROS2 node executable from source files
add_executable(aggregator_node src/aggregator_node.cpp)
# Link to the previously built library to access Aggregator classes and methods
target_link_libraries(aggregator_node aggregator_lib)

# Copy executable to installation location
install(TARGETS
  aggregator_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch files to installation location
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
