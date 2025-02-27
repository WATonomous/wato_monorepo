cmake_minimum_required(VERSION 3.10)
project(lidar_overlay)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sample_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED) # Added PCL find_package

add_library(projection_utils
  src/projection_utils.cpp
)

target_include_directories(projection_utils
    PUBLIC
    include
    projection_utils
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} # Corrected PCL includes
)

ament_target_dependencies(projection_utils
    rclcpp
    sample_msgs
    sensor_msgs
    vision_msgs
    visualization_msgs
    pcl_conversions
    pcl_ros
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    geometry_msgs
)

# Add the executable for the lidar_overlay node
add_executable(lidar_overlay_node src/lidar_overlay.cpp)

# Include directories
target_include_directories(lidar_overlay_node
    PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} # Corrected PCL includes
)

# Link libraries
target_link_libraries(lidar_overlay_node
    projection_utils
    ${OpenCV_LIBS}
    tf2::tf2
    ${PCL_LIBRARIES} #added PCL libraries
)

# Add ROS 2 dependencies
ament_target_dependencies(lidar_overlay_node
    rclcpp
    sensor_msgs
    geometry_msgs
    vision_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    cv_bridge
    pcl_conversions
    pcl_ros
)

# Install the executable
install(TARGETS
    lidar_overlay_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()