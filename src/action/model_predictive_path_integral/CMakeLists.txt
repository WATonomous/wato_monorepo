cmake_minimum_required(VERSION 3.8)
project(mppi_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Torch REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(mppi_node src/mppi_node.cpp)

# Let ament set up include dirs and link flags for rclcpp
ament_target_dependencies(mppi_node
  rclcpp
  carla_msgs
  geometry_msgs
  nav_msgs
  geometry_msgs
)

# Link against libtorch
target_link_libraries(mppi_node
  "${TORCH_LIBRARIES}"
)

# Optional safety for shared libs
set_property(TARGET mppi_node PROPERTY POSITION_INDEPENDENT_CODE ON)

install(
  TARGETS mppi_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
