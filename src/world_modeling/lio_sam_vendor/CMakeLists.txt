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
project(lio_sam_vendor)

set(LIO_SAM_GIT_REPOSITORY "https://github.com/TixiaoShan/LIO-SAM.git"
    CACHE STRING "Git repository URL")
set(LIO_SAM_GIT_TAG "08af3f32f01725372d4269838dc44c19c6d9e76b"
    CACHE STRING "Git tag/branch/commit to checkout")
set(LIO_SAM_PATCHES "" CACHE STRING "List of patch files to apply")

option(FORCE_BUILD_VENDOR_PKG "Force building LIO-SAM from source" OFF)
option(BUILD_TESTING "Build tests" ON)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
endif()

macro(build_lio_sam)
    message(STATUS "Building LIO-SAM from source")
    include(ExternalProject)

    set(LIO_SAM_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/lio_sam_install)
    set(LIO_SAM_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/lio_sam_build)
    set(LIO_SAM_VENDOR_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/opt/lio_sam_vendor")

    set(lio_sam_build_type "${CMAKE_BUILD_TYPE}")
    if(NOT lio_sam_build_type)
        set(lio_sam_build_type Release)
    endif()

    set(lio_sam_patch_arg)
    if(LIO_SAM_PATCHES)
        set(lio_sam_patch_arg PATCH_COMMAND git apply --ignore-whitespace ${LIO_SAM_PATCHES})
    endif()

    # Clone only - LIO-SAM is a colcon workspace requiring colcon build
    externalproject_add(lio_sam_src
        GIT_REPOSITORY ${LIO_SAM_GIT_REPOSITORY}
        GIT_TAG ${LIO_SAM_GIT_TAG}
        GIT_SHALLOW FALSE
        TIMEOUT 600
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        ${lio_sam_patch_arg}
    )
    externalproject_get_property(lio_sam_src SOURCE_DIR)

    # Configure cmake config and env hook files
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/lio_sam_vendor-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/lio_sam_vendor-config.cmake
        @ONLY
    )
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/env_hook/lio_sam_vendor_paths.sh.in
        ${CMAKE_CURRENT_BINARY_DIR}/lio_sam_vendor_paths.sh
        @ONLY
    )

    # Build using colcon (LIO-SAM is a ROS2 package, not a plain CMake project)
    set(LIO_SAM_BUILD_SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/build_lio_sam.bash)
    file(WRITE ${LIO_SAM_BUILD_SCRIPT}
"#!/bin/bash
set -e
unset MAKEFLAGS MFLAGS
source /opt/ros/$ENV{ROS_DISTRO}/setup.bash
cd ${SOURCE_DIR}
colcon build \\
    --base-paths . \\
    --packages-select lio_sam \\
    --merge-install \\
    --install-base ${LIO_SAM_INSTALL_DIR} \\
    --build-base ${LIO_SAM_BUILD_DIR} \\
    --cmake-args -DCMAKE_BUILD_TYPE=${lio_sam_build_type}
")

    add_custom_command(
        OUTPUT ${LIO_SAM_INSTALL_DIR}/stamp
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${LIO_SAM_INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${LIO_SAM_INSTALL_DIR}
        COMMAND /bin/bash ${LIO_SAM_BUILD_SCRIPT}
        COMMAND ${CMAKE_COMMAND} -E touch ${LIO_SAM_INSTALL_DIR}/stamp
        DEPENDS lio_sam_src
        COMMENT "Building LIO-SAM with colcon into ${LIO_SAM_INSTALL_DIR}"
    )

    add_custom_target(lio_sam_minimal_install ALL
        DEPENDS ${LIO_SAM_INSTALL_DIR}/stamp
    )

    # Install staged build into opt prefix
    install(
        DIRECTORY ${LIO_SAM_INSTALL_DIR}/
        DESTINATION ${LIO_SAM_VENDOR_INSTALL_PREFIX}
        USE_SOURCE_PERMISSIONS
    )

    # Install cmake config to both locations:
    # 1. Standard ROS 2 location for find_package to discover it
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/lio_sam_vendor-config.cmake
        DESTINATION share/${PROJECT_NAME}/cmake
    )
    # 2. Vendor-specific location (for reference)
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/lio_sam_vendor-config.cmake
        DESTINATION ${LIO_SAM_VENDOR_INSTALL_PREFIX}/cmake
    )

    # Install package.xml for downstream dependency tracking
    install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        DESTINATION share/${PROJECT_NAME}
    )

endmacro()

# Check for pre-existing LIO-SAM installations, build if none or forced
find_package(lio_sam QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(FORCE_BUILD_VENDOR_PKG OR NOT lio_sam_FOUND)
    build_lio_sam()
    ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/lio_sam_vendor_paths.sh)
else()
    message(STATUS "Found existing LIO-SAM installation: ${lio_sam_DIR}")
    message(STATUS "Skipping build. Set FORCE_BUILD_VENDOR_PKG=ON to rebuild from source.")
endif()

ament_package()
