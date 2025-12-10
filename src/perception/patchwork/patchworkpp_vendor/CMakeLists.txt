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
project(patchworkpp_vendor)

set(PATCHWORKPP_GIT_REPOSITORY "https://github.com/url-kaist/patchwork-plusplus.git" CACHE STRING "Git repository URL")
set(PATCHWORKPP_GIT_TAG "v1.0.4" CACHE STRING "Git tag/branch to checkout")
set(PATCHWORKPP_SOURCE_SUBDIR "cpp")
set(PATCHWORKPP_HEADERS_SUBDIR "patchworkpp/include")
set(PATCHWORKPP_LIBRARY_NAME "libground_seg_cores.a")

# List of patch files
set(PATCHWORKPP_PATCHES "" CACHE STRING "List of patch files to apply")

option(FORCE_BUILD_VENDOR_PKG
  "Force building Patchwork++ from source"
  OFF)

option(BUILD_TESTING "Build tests" ON)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

macro(build_patchworkpp)
    message(STATUS "Building Patchwork++ from source")
    include(ExternalProject)
    set(PATCHWORKPP_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/patchworkpp_install)
    set(PATCHWORKPP_VENDOR_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/opt/patchworkpp_vendor")

    set(patchworkpp_build_type "${CMAKE_BUILD_TYPE}")
    if(NOT patchworkpp_build_type)
        set(patchworkpp_build_type Release)
    endif()

    # Build Patchwork++
    set(patchworkpp_patch_arg)
    if(PATCHWORKPP_PATCHES)
        set(patchworkpp_patch_arg PATCH_COMMAND git apply --ignore-whitespace ${PATCHWORKPP_PATCHES})
    endif()

    externalproject_add(patchworkpp_src
        GIT_REPOSITORY ${PATCHWORKPP_GIT_REPOSITORY}
        GIT_TAG ${PATCHWORKPP_GIT_TAG}
        GIT_SHALLOW TRUE
        TIMEOUT 600
        SOURCE_SUBDIR ${PATCHWORKPP_SOURCE_SUBDIR}
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=${patchworkpp_build_type}
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            -DUSE_SYSTEM_EIGEN3=ON
            -DINCLUDE_CPP_EXAMPLES=OFF
            -DCMAKE_CXX_STANDARD=14
        INSTALL_COMMAND ""
        CONFIGURE_HANDLED_BY_BUILD TRUE
        ${patchworkpp_patch_arg}
    )
    externalproject_get_property(patchworkpp_src BINARY_DIR SOURCE_DIR)

    # Configure cmake and hook files
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/patchworkpp_vendor-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/patchworkpp_vendor-config.cmake
        @ONLY
    )
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/env_hook/patchworkpp_vendor_paths.sh.in
        ${CMAKE_CURRENT_BINARY_DIR}/patchworkpp_vendor_paths.sh
        @ONLY
    )

    # Copy required files to installation directory
    add_custom_command(
        OUTPUT ${PATCHWORKPP_INSTALL_DIR}/stamp
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${PATCHWORKPP_INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PATCHWORKPP_INSTALL_DIR}/include
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PATCHWORKPP_INSTALL_DIR}/lib
        COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${SOURCE_DIR}/${PATCHWORKPP_SOURCE_SUBDIR}/${PATCHWORKPP_HEADERS_SUBDIR}
          ${PATCHWORKPP_INSTALL_DIR}/include
        COMMAND ${CMAKE_COMMAND} -E copy
          ${BINARY_DIR}/patchworkpp/${PATCHWORKPP_LIBRARY_NAME}
          ${PATCHWORKPP_INSTALL_DIR}/lib/
        COMMAND ${CMAKE_COMMAND} -E touch ${PATCHWORKPP_INSTALL_DIR}/stamp
        DEPENDS patchworkpp_src
        COMMENT "Installing Patchwork++ to ${PATCHWORKPP_INSTALL_DIR}"
    )

    # Dummy target to run custom command by depending on output
    add_custom_target(patchworkpp_minimal_install ALL
        DEPENDS ${PATCHWORKPP_INSTALL_DIR}/stamp
    )

    # Install include and lib
    install(
        DIRECTORY ${PATCHWORKPP_INSTALL_DIR}/
        DESTINATION ${PATCHWORKPP_VENDOR_INSTALL_PREFIX}
        USE_SOURCE_PERMISSIONS
    )

    # Install cmake config
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/patchworkpp_vendor-config.cmake
        DESTINATION ${PATCHWORKPP_VENDOR_INSTALL_PREFIX}/cmake
    )

    # Install package.xml for downstream dependency tracking
    install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        DESTINATION share/${PROJECT_NAME}
    )

endmacro()

# Check for pre-existing Patchwork++ installations, build if none or forced
find_package(patchworkpp QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(FORCE_BUILD_VENDOR_PKG OR NOT patchworkpp_FOUND)
    build_patchworkpp()
    ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/patchworkpp_vendor_paths.sh)
else()
    message(STATUS "Found existing Patchwork++ installation: ${patchworkpp_CONFIG}")
    message(STATUS "Skipping build. Set FORCE_BUILD_VENDOR_PKG=ON to rebuild from source.")
endif()

# Export dependencies
ament_package()
