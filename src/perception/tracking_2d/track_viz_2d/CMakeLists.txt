cmake_minimum_required(VERSION 3.10)
project(track_viz_2d)

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
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tracking_2d_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add the executable for the track_viz_2d node
add_executable(track_viz_2d_node src/track_viz_2d.cpp)

# Include directories
target_include_directories(track_viz_2d_node
    PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(track_viz_2d_node
    ${OpenCV_LIBS}
)

# Add ROS 2 dependencies
ament_target_dependencies(track_viz_2d_node
    rclcpp
    sensor_msgs
    vision_msgs
    std_msgs
    tracking_2d_msgs
    cv_bridge
)

# Install the executable
install(TARGETS
    track_viz_2d_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()