cmake_minimum_required(VERSION 3.10)
project(hd_map)

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
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lanelet2_core REQUIRED) # Lanelet2 Core
find_package(lanelet2_io REQUIRED) # Lanelet2 IO
find_package(lanelet2_projection REQUIRED) # Lanelet2 Projection
find_package(lanelet2_routing REQUIRED) # Lanelet2 Routing

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(hd_map_lib
  src/lanelet_visualization.cpp
  src/hd_map_manager.cpp
  src/hd_map_router.cpp)

# Indicate to compiler where to search for header files
target_include_directories(hd_map_lib
  PUBLIC
  include)
# Add ROS2 dependencies required by package
ament_target_dependencies(hd_map_lib
  rclcpp
  sample_msgs
  visualization_msgs
  geometry_msgs
  sensor_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing)

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
  ament_add_gtest(sample_test test/sample_test.cpp)
  # Link to the previously built library to access Sample classes and methods
  target_link_libraries(sample_test
    hd_map_lib
    gtest_main)

  # Copy executable to installation location
  install(TARGETS
  sample_test
    DESTINATION lib/${PROJECT_NAME})
endif()

# Create ROS2 node executable from source files
add_executable(hd_map_service src/hd_map_service.cpp)
# Link to the previously built library to access Sample classes and methods
target_link_libraries(hd_map_service hd_map_lib)

# Copy executable to installation location
install(TARGETS
hd_map_service
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
