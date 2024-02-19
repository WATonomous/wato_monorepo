cmake_minimum_required(VERSION 3.5)
project(carla_config)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(path_planning_msgs REQUIRED)
find_package(embedded_msgs REQUIRED)

# Create ROS2 node executable from source files
add_executable(carla_mpc_bridge src/carla_mpc_bridge.cpp)

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

  # Create library to link cpp file with test
  add_library(mpc_bridge_lib
    src/carla_mpc_bridge.cpp)
  
  # Add ROS2 dependencies required by package
  ament_target_dependencies(mpc_bridge_lib rclcpp path_planning_msgs embedded_msgs)

  # Create test executable from source files
  ament_add_gtest(mpc_bridge_test test/mpc_bridge_test.cpp)
  # Link to dependencies
  target_link_libraries(mpc_bridge_test mpc_bridge_lib)

  # Copy executable to installation location
  install(TARGETS
    mpc_bridge_test
    DESTINATION lib/${PROJECT_NAME})
endif()

# Link with dependencies
ament_target_dependencies(carla_mpc_bridge rclcpp path_planning_msgs embedded_msgs)

# Copy executable to installation location
install(TARGETS
  carla_mpc_bridge
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()