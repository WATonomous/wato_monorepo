cmake_minimum_required(VERSION 3.10)
project(dets_2d_3d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sample_msgs REQUIRED) # Custom package containing ROS2 messages
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(projection_utils
  src/projection_utils.cpp src/cluster.cpp) # add more non-ros node files here

target_include_directories(projection_utils
  PUBLIC include)

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
  geometry_msgs)

add_executable(dets_2d_3d_node src/dets_2d_3d_node.cpp)
target_include_directories(dets_2d_3d_node
  PUBLIC include)
include(cmake/PCL.cmake)

cmake_minimum_required(VERSION 2.8)                            
find_package( OpenCV REQUIRED )                              
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries (dets_2d_3d_node ${OpenCV_LIBS})  

ament_target_dependencies(dets_2d_3d_node)
target_link_libraries(dets_2d_3d_node projection_utils)

# Copy executable to installation location
install(TARGETS
  dets_2d_3d_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch files to installation location
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
