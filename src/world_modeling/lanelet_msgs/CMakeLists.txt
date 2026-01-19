cmake_minimum_required(VERSION 3.8)
project(lanelet_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrafficLightInfo.msg"
  "msg/StopLineInfo.msg"
  "msg/Lanelet.msg"
  "msg/LaneDivider.msg"
  "msg/LanePath.msg"
  "msg/Corridor.msg"
  "msg/CurrentLaneContext.msg"
  "msg/MapVisualization.msg"
  "srv/GetRoute.srv"
  "srv/GetCorridor.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
