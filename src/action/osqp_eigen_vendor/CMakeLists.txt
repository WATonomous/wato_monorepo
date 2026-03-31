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
project(osqp_eigen_vendor)

set(OSQPEIGEN_GIT_REPOSITORY "https://github.com/robotology/osqp-eigen.git" CACHE STRING "OsqpEigen Git repository URL")
set(OSQPEIGEN_GIT_TAG "v0.8.1" CACHE STRING "OsqpEigen Git tag")

option(FORCE_BUILD_VENDOR_PKG "Force building from source" OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(osqp_vendor REQUIRED)

macro(build_osqp_eigen_vendor)
    message(STATUS "Building OsqpEigen from source")
    include(ExternalProject)

    set(OSQPEIGEN_VENDOR_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/opt/osqp_eigen_vendor")
    set(OSQPEIGEN_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/osqp_eigen_install")

    set(osqpeigen_build_type "${CMAKE_BUILD_TYPE}")
    if(NOT osqpeigen_build_type)
        set(osqpeigen_build_type Release)
    endif()

    externalproject_add(osqpeigen_src
        GIT_REPOSITORY ${OSQPEIGEN_GIT_REPOSITORY}
        GIT_TAG ${OSQPEIGEN_GIT_TAG}
        GIT_SHALLOW TRUE
        TIMEOUT 600
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=${osqpeigen_build_type}
            -DCMAKE_INSTALL_PREFIX=${OSQPEIGEN_INSTALL_DIR}
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        CONFIGURE_HANDLED_BY_BUILD TRUE
    )

    add_custom_command(
        OUTPUT ${OSQPEIGEN_INSTALL_DIR}/stamp
        COMMAND ${CMAKE_COMMAND} -E touch ${OSQPEIGEN_INSTALL_DIR}/stamp
        DEPENDS osqpeigen_src
        COMMENT "OsqpEigen build complete"
    )
    add_custom_target(osqp_eigen_vendor_install ALL
        DEPENDS ${OSQPEIGEN_INSTALL_DIR}/stamp
    )

    # Install to vendor prefix
    install(
        DIRECTORY ${OSQPEIGEN_INSTALL_DIR}/include/
        DESTINATION ${OSQPEIGEN_VENDOR_INSTALL_PREFIX}/include
    )
    install(
        DIRECTORY ${OSQPEIGEN_INSTALL_DIR}/lib/
        DESTINATION ${OSQPEIGEN_VENDOR_INSTALL_PREFIX}/lib
        USE_SOURCE_PERMISSIONS
    )

    # Configure cmake config and env hook
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/osqp_eigen_vendor-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/osqp_eigen_vendor-config.cmake
        @ONLY
    )
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/env_hook/osqp_eigen_vendor_paths.sh.in
        ${CMAKE_CURRENT_BINARY_DIR}/osqp_eigen_vendor_paths.sh
        @ONLY
    )

    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/osqp_eigen_vendor-config.cmake
        DESTINATION share/${PROJECT_NAME}/cmake
    )
    install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        DESTINATION share/${PROJECT_NAME}
    )
endmacro()

find_package(osqp_eigen_vendor QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(FORCE_BUILD_VENDOR_PKG OR NOT osqp_eigen_vendor_FOUND)
    build_osqp_eigen_vendor()
    ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/osqp_eigen_vendor_paths.sh)
else()
    message(STATUS "Found existing osqp_eigen_vendor installation, skipping build.")
endif()

ament_package()
