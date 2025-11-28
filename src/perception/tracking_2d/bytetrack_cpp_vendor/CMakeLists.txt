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
project(bytetrack_cpp_vendor)

option(FORCE_BUILD_VENDOR_PKG
  "Build ByteTrack-cpp from source, even if system-installed package is available"
  OFF)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)

macro(build_bytetrack_cpp)
    include(ExternalProject)
    set(BYTETRACK_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/bytetrack_cpp_install/)

    # Configure CMake args
    set(extra_cmake_args)

    if(DEFINED CMAKE_BUILD_TYPE)
        list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
    else()
        list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=Release)
    endif()

    list(APPEND extra_cmake_args "-DBUILD_BYTETRACK_TEST=OFF")

    # Build ByteTrack
    externalproject_add(bytetrack
        GIT_REPOSITORY https://github.com/Vertical-Beach/ByteTrack-cpp.git
        GIT_TAG main
        TIMEOUT 600
        CMAKE_ARGS ${extra_cmake_args}
        INSTALL_COMMAND ""
        CONFIGURE_HANDLED_BY_BUILD TRUE
        PATCH_COMMAND git apply --ignore-whitespace ${CMAKE_CURRENT_SOURCE_DIR}/patches/0001-class-id.patch
    )
    externalproject_get_property(bytetrack BINARY_DIR SOURCE_DIR)

    # Configure cmake and hook files
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/bytetrack_cpp_vendor-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/bytetrack_cpp_vendor-config.cmake
        @ONLY
    )
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/env_hook/bytetrack_cpp_vendor_paths.sh.in
        ${CMAKE_CURRENT_BINARY_DIR}/bytetrack_cpp_vendor_paths.sh
        @ONLY
    )

    # Copy required files to installation directory
    add_custom_command(
        OUTPUT ${BYTETRACK_INSTALL_DIR}

        # Clean up and make new directories
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${BYTETRACK_INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${BYTETRACK_INSTALL_DIR}/include
        COMMAND ${CMAKE_COMMAND} -E make_directory ${BYTETRACK_INSTALL_DIR}/lib

        # Copy headers
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${SOURCE_DIR}/include
                ${BYTETRACK_INSTALL_DIR}/include

        # Copy library
        COMMAND ${CMAKE_COMMAND} -E copy
                ${BINARY_DIR}/libbytetrack.so
                ${BYTETRACK_INSTALL_DIR}/lib/

        DEPENDS bytetrack
    )

    # Dummy target to run custom command by depending on output
    add_custom_target(bytetrack_minimal_install ALL
        DEPENDS ${BYTETRACK_INSTALL_DIR}
    )

    # Install include and lib
    install(
        DIRECTORY ${BYTETRACK_INSTALL_DIR}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/opt/bytetrack_cpp_vendor
    )

    # Install cmake config
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/bytetrack_cpp_vendor-config.cmake
        DESTINATION ${CMAKE_INSTALL_PREFIX}/opt/bytetrack_cpp_vendor/cmake
    )

endmacro()

# Check for pre-existing ByteTrack installations, build if none or forced
find_package(bytetrack QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(FORCE_BUILD_VENDOR_PKG OR NOT bytetrack_FOUND)
    build_bytetrack_cpp()
    ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/bytetrack_cpp_vendor_paths.sh)
else()
    message(STATUS "Found bytetrack in path ${bytetrack_CONFIG}")
endif()

# Export dependencies
ament_package()
