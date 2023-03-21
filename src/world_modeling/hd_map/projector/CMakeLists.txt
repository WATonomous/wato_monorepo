cmake_minimum_required(VERSION 3.10)
project(projector) # Update the project name

# ...

# Compiles source files into a library
add_library(projector_lib
  src/projector.cpp) # Update the source file name
# ...
target_include_directories(projector_lib
  PUBLIC
  include)
# Add ROS2 dependencies required by package
ament_target_dependencies(projector_lib
  rclcpp
  sample_msgs)

# ...

# Create ROS2 node executable from source files
add_executable(projector_node src/projector_node.cpp) # Update the source file name
# Link to the previously built library to access Projector classes and methods
target_link_libraries(projector_node projector_lib) # Update the library name

# Copy executable to installation location
install(TARGETS
projector_node
  DESTINATION lib/${PROJECT_NAME})

# ...

ament_package()
