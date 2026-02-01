cmake_minimum_required(VERSION 3.10)
project(behaviortree_ros2_vendor)

set(BEHAVIORTREE_ROS2_GIT_REPOSITORY "https://github.com/BehaviorTree/BehaviorTree.ROS2.git"
    CACHE STRING "Git repository URL")
set(BEHAVIORTREE_ROS2_GIT_TAG "6c6aa078ee7bc52fec98984bed4964556abf5beb"
    CACHE STRING "Git tag/branch/commit to checkout")
set(BEHAVIORTREE_ROS2_PATCHES "" CACHE STRING "List of patch files to apply")

option(FORCE_BUILD_VENDOR_PKG "Force building BehaviorTree.ROS2 from source" OFF)
option(BUILD_TESTING "Build tests" ON)

find_package(ament_cmake REQUIRED)
include(ExternalProject)

if(BUILD_TESTING)
endif()

macro(build_behaviortree_ros2)
  message(STATUS "Building BehaviorTree.ROS2 from source")

  set(BT_ROS2_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/behaviortree_ros2_install)
  set(BT_ROS2_VENDOR_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/opt/behaviortree_ros2_vendor")

  set(bt_ros2_build_type "${CMAKE_BUILD_TYPE}")
  if(NOT bt_ros2_build_type)
    set(bt_ros2_build_type Release)
  endif()

  set(bt_ros2_patch_arg)
  if(BEHAVIORTREE_ROS2_PATCHES)
    set(bt_ros2_patch_arg PATCH_COMMAND git apply --ignore-whitespace ${BEHAVIORTREE_ROS2_PATCHES})
  endif()

  # Clone only
  ExternalProject_Add(behaviortree_ros2_src
    GIT_REPOSITORY ${BEHAVIORTREE_ROS2_GIT_REPOSITORY}
    GIT_TAG ${BEHAVIORTREE_ROS2_GIT_TAG}
    GIT_SHALLOW TRUE
    TIMEOUT 600
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    ${bt_ros2_patch_arg}
  )
  ExternalProject_Get_Property(behaviortree_ros2_src SOURCE_DIR)

  # Generate a CMake script that will run the bash+colcon command reliably
  set(_build_script_in  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_behaviortree_ros2.cmake.in")
  set(_build_script_out "${CMAKE_CURRENT_BINARY_DIR}/build_behaviortree_ros2.cmake")
  configure_file(${_build_script_in} ${_build_script_out} @ONLY)

  add_custom_command(
    OUTPUT ${BT_ROS2_INSTALL_DIR}/stamp
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${BT_ROS2_INSTALL_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${BT_ROS2_INSTALL_DIR}
    COMMAND ${CMAKE_COMMAND} -P ${_build_script_out}
    COMMAND ${CMAKE_COMMAND} -E touch ${BT_ROS2_INSTALL_DIR}/stamp
    DEPENDS behaviortree_ros2_src
    COMMENT "Building and staging BehaviorTree.ROS2 into ${BT_ROS2_INSTALL_DIR}"
  )

  add_custom_target(behaviortree_ros2_minimal_install ALL
    DEPENDS ${BT_ROS2_INSTALL_DIR}/stamp
  )

  # Copy staged install into opt prefix
  install(
    DIRECTORY ${BT_ROS2_INSTALL_DIR}/
    DESTINATION ${BT_ROS2_VENDOR_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
  )

  # Env hook
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/env_hook/behaviortree_ros2_vendor_paths.sh.in
    ${CMAKE_CURRENT_BINARY_DIR}/behaviortree_ros2_vendor_paths.sh
    @ONLY
  )
  ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/behaviortree_ros2_vendor_paths.sh)

  install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    DESTINATION share/${PROJECT_NAME}
  )
endmacro()

# In CI you can remove this and always build, but leaving it is fine
find_package(behaviortree_ros2 QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(FORCE_BUILD_VENDOR_PKG OR NOT behaviortree_ros2_FOUND)
  build_behaviortree_ros2()
else()
  message(STATUS "Found existing behaviortree_ros2 installation: ${behaviortree_ros2_DIR}")
  message(STATUS "Skipping build. Set FORCE_BUILD_VENDOR_PKG=ON to rebuild from source.")
endif()

ament_package()
