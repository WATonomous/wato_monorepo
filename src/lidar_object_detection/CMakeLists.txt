cmake_minimum_required(VERSION 3.5)
project(lidar_object_detection)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Debug") 
# SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "-std=c++14 -O3 -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -O3 -Wall")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  std_msgs
  common_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_geometry_msgs
  CUDA
  visualization_msgs
  )

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

include(cmake/PCL.cmake)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE CUDA_SRCS "*.cu")

cuda_add_executable(
  lidar_object_detection 
    main.cpp 
    ${SOURCE_FILES} 
    ${ALL_SRCS} 
    ${CUDA_SRCS}
)

target_link_libraries(
  lidar_object_detection
  libnvinfer.so
  libnvonnxparser.so
  # ${Boost_SYSTEM_LIBRARIES} 
  # ${PCL_LIBRARIES}
  # ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES}
)

ament_target_dependencies(lidar_object_detection
  rclcpp
  std_msgs
  common_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS
  lidar_object_detection
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# include(cmake/PCL.cmake)
include(cmake/pprt.cmake)

ament_package()
