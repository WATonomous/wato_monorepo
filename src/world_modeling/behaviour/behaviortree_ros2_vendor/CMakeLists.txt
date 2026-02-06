# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
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

if(BUILD_TESTING)
endif()

macro(build_behaviortree_ros2)
    message(STATUS "Building BehaviorTree.ROS2 from source")
    include(ExternalProject)

    set(BT_ROS2_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/behaviortree_ros2_install)
    set(BT_ROS2_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/behaviortree_ros2_build)
    set(BT_ROS2_VENDOR_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/opt/behaviortree_ros2_vendor")

    set(bt_ros2_build_type "${CMAKE_BUILD_TYPE}")
    if(NOT bt_ros2_build_type)
        set(bt_ros2_build_type Release)
    endif()

    set(bt_ros2_patch_arg)
    if(BEHAVIORTREE_ROS2_PATCHES)
        set(bt_ros2_patch_arg PATCH_COMMAND git apply --ignore-whitespace ${BEHAVIORTREE_ROS2_PATCHES})
    endif()

    # Clone only - BehaviorTree.ROS2 is a colcon workspace requiring colcon build
    externalproject_add(behaviortree_ros2_src
        GIT_REPOSITORY ${BEHAVIORTREE_ROS2_GIT_REPOSITORY}
        GIT_TAG ${BEHAVIORTREE_ROS2_GIT_TAG}
        GIT_SHALLOW TRUE
        TIMEOUT 600
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        ${bt_ros2_patch_arg}
    )
    externalproject_get_property(behaviortree_ros2_src SOURCE_DIR)

    # Configure cmake config and env hook files
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/behaviortree_ros2_vendor-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/behaviortree_ros2_vendor-config.cmake
        @ONLY
    )
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/env_hook/behaviortree_ros2_vendor_paths.sh.in
        ${CMAKE_CURRENT_BINARY_DIR}/behaviortree_ros2_vendor_paths.sh
        @ONLY
    )

    # Build using colcon (BehaviorTree.ROS2 is a ROS2 workspace, not a plain CMake project)
    # Create a build script to avoid shell compatibility issues
    set(BT_ROS2_BUILD_SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/build_bt_ros2.bash)
    file(WRITE ${BT_ROS2_BUILD_SCRIPT}
"#!/bin/bash
set -e
unset MAKEFLAGS MFLAGS
source /opt/ros/$ENV{ROS_DISTRO}/setup.bash
cd ${SOURCE_DIR}
colcon build \\
    --base-paths . \\
    --packages-select behaviortree_ros2 btcpp_ros2_interfaces \\
    --merge-install \\
    --install-base ${BT_ROS2_INSTALL_DIR} \\
    --build-base ${BT_ROS2_BUILD_DIR} \\
    --cmake-args -DCMAKE_BUILD_TYPE=${bt_ros2_build_type}
")

    add_custom_command(
        OUTPUT ${BT_ROS2_INSTALL_DIR}/stamp
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${BT_ROS2_INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${BT_ROS2_INSTALL_DIR}
        COMMAND /bin/bash ${BT_ROS2_BUILD_SCRIPT}
        COMMAND ${CMAKE_COMMAND} -E touch ${BT_ROS2_INSTALL_DIR}/stamp
        DEPENDS behaviortree_ros2_src
        COMMENT "Building BehaviorTree.ROS2 with colcon into ${BT_ROS2_INSTALL_DIR}"
    )

    add_custom_target(behaviortree_ros2_minimal_install ALL
        DEPENDS ${BT_ROS2_INSTALL_DIR}/stamp
    )

    # Install staged build into opt prefix
    install(
        DIRECTORY ${BT_ROS2_INSTALL_DIR}/
        DESTINATION ${BT_ROS2_VENDOR_INSTALL_PREFIX}
        USE_SOURCE_PERMISSIONS
    )

    # Install cmake config to both locations:
    # 1. Standard ROS 2 location for find_package to discover it
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/behaviortree_ros2_vendor-config.cmake
        DESTINATION share/${PROJECT_NAME}/cmake
    )
    # 2. Vendor-specific location (for reference)
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/behaviortree_ros2_vendor-config.cmake
        DESTINATION ${BT_ROS2_VENDOR_INSTALL_PREFIX}/cmake
    )

    # Install package.xml for downstream dependency tracking
    install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        DESTINATION share/${PROJECT_NAME}
    )

endmacro()

# Check for pre-existing BehaviorTree.ROS2 installations, build if none or forced
find_package(behaviortree_ros2 QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(FORCE_BUILD_VENDOR_PKG OR NOT behaviortree_ros2_FOUND)
    build_behaviortree_ros2()
    ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/behaviortree_ros2_vendor_paths.sh)
else()
    message(STATUS "Found existing BehaviorTree.ROS2 installation: ${behaviortree_ros2_DIR}")
    message(STATUS "Skipping build. Set FORCE_BUILD_VENDOR_PKG=ON to rebuild from source.")
endif()

ament_package()
