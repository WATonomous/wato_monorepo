cmake_minimum_required(VERSION 3.8)
set(ProjectName "lane_detection")
project(${ProjectName})


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### TensorRT model ###

# Select build system and set compile options
include(${CMAKE_CURRENT_LIST_DIR}/src/tensorrt_model/common_helper/cmakes/build_setting.cmake)

add_executable(lane_detection src/lane_detection_node.cpp)

# Link ImageProcessor module
add_subdirectory(./src/tensorrt_model/ultra_fast_lane_detection_v2/image_processor image_processor)
target_include_directories(${ProjectName} PUBLIC ./src/tensorrt_model/ultra_fast_lane_detection_v2/image_processor)
target_link_libraries(${ProjectName} ImageProcessor)

# For OpenCV
find_package(OpenCV REQUIRED)
target_include_directories(${ProjectName} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${ProjectName} ${OpenCV_LIBS})

# Copy resouce
file(COPY ${CMAKE_CURRENT_LIST_DIR}/src/tensorrt_model/ultra_fast_lane_detection_v2/resource/model DESTINATION ${CMAKE_BINARY_DIR}/resource)
add_definitions(-DRESOURCE_DIR="${CMAKE_BINARY_DIR}/resource/")


### End of TensorRT model ###

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)


install(TARGETS
lane_detection
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(lane_detection rclcpp OpenCV std_msgs sensor_msgs cv_bridge image_transport)
ament_package()
