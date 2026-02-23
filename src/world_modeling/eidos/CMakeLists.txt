cmake_minimum_required(VERSION 3.8)
project(eidos)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eidos_msgs REQUIRED)
find_package(pluginlib REQUIRED)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# ---- Core library ----
add_library(eidos_core SHARED
  src/slam_core.cpp
  src/pose_graph.cpp
  src/map_manager.cpp
  src/scan_matcher.cpp
)
target_include_directories(eidos_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${pcl_conversions_INCLUDE_DIRS}
)
target_link_libraries(eidos_core PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${eidos_msgs_TARGETS}
  pluginlib::pluginlib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
  OpenMP::OpenMP_CXX
)

# ---- Plugin library ----
add_library(eidos_plugins SHARED
  src/plugins/lidar_kep_factor.cpp
  src/plugins/imu_integration_factor.cpp
  src/plugins/gps_factor.cpp
  src/plugins/euclidean_distance_loop_closure_factor.cpp
  src/plugins/gps_icp_relocalization.cpp
)
target_include_directories(eidos_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(eidos_plugins PUBLIC
  eidos_core
)

# ---- Node executable ----
add_executable(eidos_node src/slam_node.cpp)
target_include_directories(eidos_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(eidos_node PRIVATE
  eidos_core
)

# ---- Plugin descriptions ----
pluginlib_export_plugin_description_file(eidos factor_plugins.xml)
pluginlib_export_plugin_description_file(eidos relocalization_plugins.xml)

# ---- Install ----
install(TARGETS eidos_core eidos_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS eidos_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES factor_plugins.xml relocalization_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(wato_test REQUIRED)

  add_wato_test(test_pose_graph
    test/test_pose_graph.cpp
    LIBRARIES
      eidos_core
  )

  add_wato_test(test_map_manager
    test/test_map_manager.cpp
    LIBRARIES
      eidos_core
  )

  add_wato_test(test_scan_matcher
    test/test_scan_matcher.cpp
    LIBRARIES
      eidos_core
  )
endif()

# ---- Export ----
ament_export_include_directories(include)
ament_export_libraries(eidos_core eidos_plugins)
ament_export_dependencies(
  rclcpp sensor_msgs nav_msgs
  tf2 tf2_ros pcl_conversions eidos_msgs pluginlib
)

ament_package()
