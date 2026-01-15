cmake_minimum_required(VERSION 3.10)
project(carla_ros_bridge)

find_package(ament_cmake REQUIRED)

# Meta-package has no build targets
# Just depends on all other packages

ament_package()
