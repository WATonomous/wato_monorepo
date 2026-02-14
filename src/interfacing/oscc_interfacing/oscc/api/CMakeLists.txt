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
cmake_minimum_required(VERSION 3.8)

project(osccapi)

# Include OSCC configuration for vehicle selection
include(${CMAKE_CURRENT_SOURCE_DIR}/OsccConfig.cmake)

set(INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/oscc.c)

set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE C)

set(OBJECTS ${PROJECT_NAME}_objects)
set(SHARED_LIB ${PROJECT_NAME}_shared_lib)
set(STATIC_LIB ${PROJECT_NAME}_static_lib)

# Reuse object files for both shared and static libraries
# rather than recompiling for both
add_library(${OBJECTS} OBJECT ${SOURCES})
target_include_directories(${OBJECTS} PUBLIC ${INCLUDES})
set_target_properties(${OBJECTS} PROPERTIES POSITION_INDEPENDENT_CODE 1)

add_library(${SHARED_LIB} SHARED $<TARGET_OBJECTS:${OBJECTS}>)
set_target_properties(${SHARED_LIB} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(${SHARED_LIB} PUBLIC
  $<BUILD_INTERFACE:${INCLUDES}>
  $<INSTALL_INTERFACE:include>
)

add_library(${STATIC_LIB} STATIC $<TARGET_OBJECTS:${OBJECTS}>)
target_include_directories(${STATIC_LIB} PUBLIC
  $<BUILD_INTERFACE:${INCLUDES}>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(${STATIC_LIB} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
