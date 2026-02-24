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
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eidos_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp_vendor REQUIRED)
find_package(small_gicp REQUIRED)

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
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${eidos_msgs_TARGETS}
  pluginlib::pluginlib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
  OpenMP::OpenMP_CXX
)

# ---- Plugin library ----
add_library(eidos_plugins SHARED
  src/plugins/factors/lidar_kep_factor.cpp
  src/plugins/factors/imu_integration_factor.cpp
  src/plugins/factors/gps_factor.cpp
  src/plugins/factors/euclidean_distance_loop_closure_factor.cpp
  src/plugins/relocalization/gps_icp_relocalization.cpp
  src/plugins/visualization/keyframe_map_visualization.cpp
  src/plugins/visualization/gps_visualization.cpp
  src/plugins/visualization/loop_closure_visualization.cpp
  src/plugins/visualization/keyframe_pose_visualization.cpp
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
  small_gicp::small_gicp
  ${visualization_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

# ---- Node executable ----
add_executable(eidos_node src/main.cpp)
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
pluginlib_export_plugin_description_file(eidos visualization_plugins.xml)

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

install(FILES factor_plugins.xml relocalization_plugins.xml visualization_plugins.xml
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
  rclcpp rclcpp_lifecycle sensor_msgs nav_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions eidos_msgs pluginlib
)

ament_package()
