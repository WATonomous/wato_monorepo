cmake_minimum_required(VERSION 3.10)
project(camera_compression)

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
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create ROS2 node executable from source files
add_executable(camera_compression_node src/camera_compression_node.cpp)

ament_target_dependencies(camera_compression_node rclcpp image_transport cv_bridge sensor_msgs)

target_include_directories(camera_compression_node PUBLIC include)

# Copy executable to installation location
install(TARGETS
  camera_compression_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch files to installation location
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
