cmake_minimum_required(VERSION 3.10)
project(spatial_association)

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
find_package(camera_object_detection_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED) 

add_library(projection_utils
  utils/src/projection_utils.cpp
)

target_include_directories(projection_utils
    PRIVATE
    utils/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
)

target_link_libraries(projection_utils
    PUBLIC
        rclcpp::rclcpp
        tf2::tf2
        tf2_ros::tf2_ros
        tf2_geometry_msgs::tf2_geometry_msgs
        tf2_eigen::tf2_eigen
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
    PRIVATE
        ${sample_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${vision_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        ${camera_object_detection_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${pcl_conversions_TARGETS}
        ${pcl_ros_TARGETS}
)


add_library(spatial_association_core
  src/spatial_association_core.cpp
)

target_include_directories(spatial_association_core
    PRIVATE
    include
    utils/include
    ${PCL_INCLUDE_DIRS}
)


target_link_libraries(spatial_association_core
    PRIVATE
        projection_utils
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${vision_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
    PUBLIC
        ${PCL_LIBRARIES}
)


add_executable(spatial_association_node src/spatial_association.cpp)

target_include_directories(spatial_association_node
    PRIVATE
    include
    utils/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
)


target_link_libraries(spatial_association_node
    PRIVATE
        spatial_association_core
        projection_utils
        ${pcl_conversions_TARGETS}
        ${pcl_ros_TARGETS}
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${vision_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${camera_object_detection_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
    PUBLIC
        rclcpp::rclcpp
        tf2::tf2
        tf2_ros::tf2_ros
        tf2_geometry_msgs::tf2_geometry_msgs
        cv_bridge::cv_bridge
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

# Install the executable and libraries
install(TARGETS
    spatial_association_node
    spatial_association_core
    projection_utils
    DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
    DESTINATION include
)

# Install utils header files
install(DIRECTORY utils/include/
    DESTINATION include
)


install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()
