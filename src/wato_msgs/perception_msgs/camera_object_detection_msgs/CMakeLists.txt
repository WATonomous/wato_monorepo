cmake_minimum_required(VERSION 3.8)
project(camera_object_detection_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/BatchDetection.msg"
    DEPENDENCIES std_msgs sensor_msgs vision_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()