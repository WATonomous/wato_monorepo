cmake_minimum_required(VERSION 3.10)
project(carla_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/ScenarioStatus.msg"
)

set(srv_files
  "srv/SwitchScenario.srv"
  "srv/GetAvailableScenarios.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
