cmake_minimum_required(VERSION 3.10)
project(localization)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

include_directories(
  SYSTEM /usr/include/eigen3
)

# Include GTSAM and project headers
include_directories(
  ${GTSAM_INCLUDE_DIR}/gtsam
  ${GTSAM_INCLUDE_DIRS}
)

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(sample_lib
  src/sample.cpp)
# Indicate to compiler where to search for header files
target_include_directories(sample_lib
  PUBLIC include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIR}/gtsam
  ${GTSAM_INCLUDE_DIR})
# Add ROS2 dependencies required by package
ament_target_dependencies(sample_lib rclcpp sample_msgs sensor_msgs geometry_msgs nav_msgs GeographicLib GTSAM)

# By default tests are built. This can be overridden by modifying the
# colcon build command to include the -DBUILD_TESTING=OFF flag.
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  # Search for dependencies required for building tests + linting
  find_package(ament_cmake_gtest REQUIRED)

  # Create test executable from source files
  ament_add_gtest(sample_test test/sample_test.cpp)
  # Link to the previously built library to access Localization classes and methods
  target_link_libraries(sample_test sample_lib ${GeographicLib_LIBRARIES} gtsam)

  # Copy executable to installation location
  install(TARGETS
    sample_test
    DESTINATION lib/${PROJECT_NAME})
endif()

# Create ROS2 node executable from source files
add_executable(localization_node src/localization_node.cpp)

# Link to the previously built library to access Localization classes and methods
target_link_libraries(localization_node sample_lib ${GeographicLib_LIBRARIES} gtsam)

# Copy executable to installation location
install(TARGETS
  localization_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch files to installation location
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
