cmake_minimum_required(VERSION 3.8)
project(lidar_object_detection)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Debug") 
# SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "-std=c++14 -O3 -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -O3 -Wall")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CUDA REQUIRED)
#find_package(pcl_conversions REQUIRED)
#find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


file(GLOB_RECURSE POINTPILLAR_CPP "src/pointpillars/*.cpp")
file(GLOB_RECURSE PONTPILLAR_CU "src/pointpillars/*.cu")

include_directories(src/pointpillars/include)

cuda_add_executable(
  lidar_object_detection
  src/lidar_detection_node.cpp
    ${POINTPILLAR_CPP}
    ${PONTPILLAR_CU}
)

# add_executable(lidar_object_detection src/lidar_detection_node.cpp)

target_link_libraries(
  lidar_object_detection
  libnvinfer.so
  libnvonnxparser.so
  # ${Boost_SYSTEM_LIBRARIES} 
  # ${PCL_LIBRARIES}
  # ${PCL_COMMON_LIB RARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES}
)

ament_target_dependencies(lidar_object_detection rclcpp std_msgs)

install(TARGETS
  lidar_object_detection
  DESTINATION lib/${PROJECT_NAME})

ament_package()
