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
cmake_minimum_required(VERSION 3.14)
project(behaviortree_ros2_vendor)

# -----------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------
set(BT_ROS2_GIT_URL "https://github.com/BehaviorTree/BehaviorTree.ROS2.git" CACHE STRING "Git repository URL")
# The specific commit hash you requested
set(BT_ROS2_GIT_TAG "6c6aa078ee7bc52fec98984bed4964556abf5beb" CACHE STRING "Git tag/branch to checkout")

option(FORCE_BUILD_VENDOR_PKG "Force building from source" OFF)
option(BUILD_TESTING "Build tests" ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# -----------------------------------------------------------------------
# Build Logic
# -----------------------------------------------------------------------
macro(build_bt_ros2)
    message(STATUS "Building BehaviorTree.ROS2 from source...")
    include(ExternalProject)

    set(BT_ROS2_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/bt_ros2_install)
    set(BT_ROS2_VENDOR_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/opt/${PROJECT_NAME}")

    set(bt_ros2_build_type "${CMAKE_BUILD_TYPE}")
    if(NOT bt_ros2_build_type)
        set(bt_ros2_build_type Release)
    endif()

    # We use CMAKE_ARGS to ensure the inner project sees our current ROS environment
    externalproject_add(bt_ros2_src
        GIT_REPOSITORY ${BT_ROS2_GIT_URL}
        GIT_TAG ${BT_ROS2_GIT_TAG}
        GIT_SHALLOW TRUE
        TIMEOUT 600
        # No SOURCE_SUBDIR needed, root is source
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=${bt_ros2_build_type}
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            -DCMAKE_CXX_STANDARD=17
            # Pass the current prefix path so it finds rclcpp and behaviortree_cpp
            -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
            -DCMAKE_INSTALL_PREFIX=${BT_ROS2_INSTALL_DIR}
            -DBUILD_TESTING=OFF
            -DBUILD_EXAMPLES=OFF
    )

    # 1. Configure the config and env hook files
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        @ONLY
    )
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/env_hook/${PROJECT_NAME}_paths.sh.in
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_paths.sh
        @ONLY
    )

    # 2. Custom install command
    # Because BT.ROS2 is a full CMake project, we let it install itself to BT_ROS2_INSTALL_DIR 
    # via the CMAKE_INSTALL_PREFIX arg above.
    # Then we copy that installation to our vendor location.
    
    add_custom_command(
        OUTPUT ${BT_ROS2_INSTALL_DIR}/stamp
        # Clean destination
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${BT_ROS2_INSTALL_DIR}
        # Run the install manually for the external project if needed, 
        # but ExternalProject usually handles "make install" if INSTALL_COMMAND is not overridden.
        # We will touch a stamp file to signal completion.
        COMMAND ${CMAKE_COMMAND} -E touch ${BT_ROS2_INSTALL_DIR}/stamp
        DEPENDS bt_ros2_src
        COMMENT "Built BehaviorTree.ROS2"
    )

    add_custom_target(bt_ros2_minimal_install ALL
        DEPENDS ${BT_ROS2_INSTALL_DIR}/stamp
    )

    # 3. Install Step: Copy the artifacts from build dir to system install
    
    # Install include headers
    install(
        DIRECTORY ${BT_ROS2_INSTALL_DIR}/include/
        DESTINATION ${BT_ROS2_VENDOR_INSTALL_PREFIX}/include
        USE_SOURCE_PERMISSIONS
    )

    # Install libraries
    install(
        DIRECTORY ${BT_ROS2_INSTALL_DIR}/lib/
        DESTINATION ${BT_ROS2_VENDOR_INSTALL_PREFIX}/lib
        USE_SOURCE_PERMISSIONS
    )

    # Install cmake config
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        DESTINATION share/${PROJECT_NAME}/cmake
    )
    
    # Also install the vendor specific config for internal lookups
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        DESTINATION ${BT_ROS2_VENDOR_INSTALL_PREFIX}/cmake
    )

    install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        DESTINATION share/${PROJECT_NAME}
    )

endmacro()

# -----------------------------------------------------------------------
# Main Execution
# -----------------------------------------------------------------------

# Check if we need to build
# Note: Since this is a vendor package for a specific commit, we usually force build 
# or check if the specific target exists. For now, we default to building.
build_bt_ros2()
ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_paths.sh)

ament_package()