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
project(cuda_bevfusion_vendor)

set(CUDA_BEVFUSION_GIT_REPOSITORY "https://github.com/WATonomous/wato-cuda-bevfusion.git" CACHE STRING "Git repository URL")
set(CUDA_BEVFUSION_GIT_TAG "master" CACHE STRING "Git tag/branch to checkout")

option(FORCE_BUILD_VENDOR_PKG
  "Force building cuda-bevfusion from source"
  OFF)

find_package(ament_cmake REQUIRED)

macro(build_cuda_bevfusion)
    message(STATUS "Building cuda-bevfusion from source")
    include(ExternalProject)
    set(BEVFUSION_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/cuda_bevfusion_install)
    set(BEVFUSION_VENDOR_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/opt/cuda_bevfusion_vendor")

    set(bevfusion_build_type "${CMAKE_BUILD_TYPE}")
    if(NOT bevfusion_build_type)
        set(bevfusion_build_type Release)
    endif()

    # --- Set Environment Variables required by cuda-bevfusion ---
    if(NOT DEFINED ENV{CUDASM})
      set(ENV{CUDASM} "89")
    endif()
    if(NOT DEFINED ENV{SPCONV_CUDA_VERSION})
      set(ENV{SPCONV_CUDA_VERSION} "12.8")
    endif()
    if(NOT DEFINED ENV{CUDA_Inc})
      set(ENV{CUDA_Inc} "/usr/local/cuda/include")
    endif()
    if(NOT DEFINED ENV{CUDA_Lib})
      set(ENV{CUDA_Lib} "/usr/local/cuda/lib64")
    endif()
    if(NOT DEFINED ENV{TensorRT_Inc})
      set(ENV{TensorRT_Inc} "/usr/include/x86_64-linux-gnu")
    endif()
    if(NOT DEFINED ENV{TensorRT_Lib})
      set(ENV{TensorRT_Lib} "/usr/lib/x86_64-linux-gnu")
    endif()

    # Build cuda-bevfusion using ExternalProject
    externalproject_add(cuda_bevfusion_src
        GIT_REPOSITORY ${CUDA_BEVFUSION_GIT_REPOSITORY}
        GIT_TAG ${CUDA_BEVFUSION_GIT_TAG}
        TIMEOUT 600

        # Specify the source directory subdirectory
        SOURCE_SUBDIR CUDA-BEVFusion

        # 1. Initialize git-lfs and pull the binary files (like libspconv.so)
        # 2. Patch make_pb.sh to fallback to system protoc
        # 3. Run the protobuf generation script
        PATCH_COMMAND git lfs install && git lfs pull
              COMMAND sed -i "s|protoc=\\\$CONDA_PREFIX/bin/protoc|protoc=protoc|g" <SOURCE_DIR>/CUDA-BEVFusion/src/onnx/make_pb.sh
              COMMAND bash <SOURCE_DIR>/CUDA-BEVFusion/src/onnx/make_pb.sh


        # Explicitly configure using 'cmake -E env' to pass build-time environment variables
        CONFIGURE_COMMAND ${CMAKE_COMMAND} -E env
            CUDASM=$ENV{CUDASM}
            SPCONV_CUDA_VERSION=$ENV{SPCONV_CUDA_VERSION}
            CUDA_Inc=$ENV{CUDA_Inc}
            CUDA_Lib=$ENV{CUDA_Lib}
            TensorRT_Inc=$ENV{TensorRT_Inc}
            TensorRT_Lib=$ENV{TensorRT_Lib}
            ${CMAKE_COMMAND} -H<SOURCE_DIR>/CUDA-BEVFusion -B<BINARY_DIR>
            -DCMAKE_BUILD_TYPE=${bevfusion_build_type}
            -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}

        # Explicitly build using 'cmake -E env' to ensure NVCC gets CUDASM during compilation
        BUILD_COMMAND ${CMAKE_COMMAND} -E env
            CUDASM=$ENV{CUDASM}
            SPCONV_CUDA_VERSION=$ENV{SPCONV_CUDA_VERSION}
            CUDA_Inc=$ENV{CUDA_Inc}
            CUDA_Lib=$ENV{CUDA_Lib}
            TensorRT_Inc=$ENV{TensorRT_Inc}
            TensorRT_Lib=$ENV{TensorRT_Lib}
            ${CMAKE_COMMAND} --build <BINARY_DIR>

        INSTALL_COMMAND ""
        CONFIGURE_HANDLED_BY_BUILD TRUE

        # Ensure submodules are recursively cloned
        GIT_SUBMODULES "CUDA-BEVFusion/bevfusion dependencies/stb dependencies/dlpack libraries/3DSparseConvolution libraries/cuOSD"
    )

    externalproject_get_property(cuda_bevfusion_src BINARY_DIR SOURCE_DIR)

    # Configure cmake and hook files
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cuda_bevfusion_vendor-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/cuda_bevfusion_vendor-config.cmake
        @ONLY
    )
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/env_hook/cuda_bevfusion_vendor_paths.sh.in
        ${CMAKE_CURRENT_BINARY_DIR}/cuda_bevfusion_vendor_paths.sh
        @ONLY
    )

    # Copy files to install folder via custom command (triggered only when source updates)
    add_custom_command(
        OUTPUT ${BEVFUSION_INSTALL_DIR}/stamp
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${BEVFUSION_INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${BEVFUSION_INSTALL_DIR}/include
        COMMAND ${CMAKE_COMMAND} -E make_directory ${BEVFUSION_INSTALL_DIR}/lib

        # Copy compiled libraries
        COMMAND ${CMAKE_COMMAND} -E copy
                ${BINARY_DIR}/libbevfusion_core.so
                ${BINARY_DIR}/libcustom_layernorm.so
                ${BEVFUSION_INSTALL_DIR}/lib/

        # Copy headers from the CUDA-BEVFusion/src directory
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${SOURCE_DIR}/CUDA-BEVFusion/src
                ${BEVFUSION_INSTALL_DIR}/include

        COMMAND ${CMAKE_COMMAND} -E touch ${BEVFUSION_INSTALL_DIR}/stamp
        DEPENDS cuda_bevfusion_src
        COMMENT "Installing cuda-bevfusion to ${BEVFUSION_INSTALL_DIR}"
    )

    # Dummy target to run custom command by depending on output
    add_custom_target(cuda_bevfusion_minimal_install ALL
        DEPENDS ${BEVFUSION_INSTALL_DIR}/stamp
    )

    # Install include and lib under /opt
    install(
        DIRECTORY ${BEVFUSION_INSTALL_DIR}/
        DESTINATION ${BEVFUSION_VENDOR_INSTALL_PREFIX}
        USE_SOURCE_PERMISSIONS
    )

    # Install cmake config for discovery
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/cuda_bevfusion_vendor-config.cmake
        DESTINATION share/${PROJECT_NAME}/cmake
    )

    # Install package.xml for downstream dependency tracking
    install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        DESTINATION share/${PROJECT_NAME}
    )

endmacro()

# Check for pre-existing installations, build if none or forced
find_package(cuda_bevfusion QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(FORCE_BUILD_VENDOR_PKG OR NOT cuda_bevfusion_FOUND)
    build_cuda_bevfusion()
    ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/cuda_bevfusion_vendor_paths.sh)
else()
    message(STATUS "Found existing cuda_bevfusion: ${cuda_bevfusion_CONFIG}")
endif()

# Export dependencies
ament_package(CONFIG_EXTRAS
    "${CMAKE_CURRENT_BINARY_DIR}/cuda_bevfusion_vendor-config.cmake"
)
