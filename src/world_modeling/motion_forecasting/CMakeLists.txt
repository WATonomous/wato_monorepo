cmake_minimum_required(VERSION 3.14)
project(motion_forecasting)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

find_package(glog REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(motion_forecasting_node SHARED
  src/motion_forecasting_node.cpp
  src/path_generator.cpp
  src/debug.cpp
)

target_link_libraries(motion_forecasting_node glog::glog)

rclcpp_components_register_node(motion_forecasting_node
  PLUGIN "motion_forecasting::MapBasedPredictionNode"
  PLUGIN "autoware_interface::TrafficLightConverterNode"
  EXECUTABLE motion_forecasting
)

## Tests
if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE test_files test/**/*.cpp)
  ament_add_ros_isolated_gtest(test_motion_forecasting ${test_files})

  target_link_libraries(test_motion_forecasting
  motion_forecasting_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
