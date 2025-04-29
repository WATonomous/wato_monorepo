cmake_minimum_required(VERSION 3.8)
project(tracking_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedDetection3D.msg"
  "msg/TrackedDetection3DArray.msg"
  "msg/Obstacle.msg"
  "msg/TrackedObstacle.msg"
  "msg/TrackedObstacleList.msg"
  "msg/TrackedObstacleState.msg"
  "msg/ObstacleList.msg"
  DEPENDENCIES std_msgs vision_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()