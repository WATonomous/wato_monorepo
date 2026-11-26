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

# Camera trigger firmware. One source tree, two boards:
#   cmake --preset f767 && cmake --build --preset f767   -> NUCLEO-F767ZI firmware
#   cmake --preset h753 && cmake --build --preset h753   -> NUCLEO-H753ZI firmware
#   cmake --preset host && cmake --build --preset host && ctest --preset host   -> unit tests
# This is a standalone CMake project, not a ROS package (colcon ignores it: COLCON_IGNORE).

cmake_minimum_required(VERSION 3.20)
project(camera_sync C ASM)

set(CS_BOARD "" CACHE STRING "nucleo_f767zi or nucleo_h753zi. Empty builds the host unit tests.")

set(CS_APP_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/app/src/cs_sched.c
  ${CMAKE_CURRENT_SOURCE_DIR}/app/src/cs_report.c
  ${CMAKE_CURRENT_SOURCE_DIR}/app/src/cs_net.c
)
set(CS_APP_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/app/include
  ${CMAKE_CURRENT_SOURCE_DIR}/config
)
set(CS_WARNINGS -Wall -Wextra -Wshadow -Wdouble-promotion -Wformat=2 -Wundef -Werror)

if(CS_BOARD STREQUAL "")
  include(cmake/host_tests.cmake)
else()
  include(cmake/firmware.cmake)
endif()
