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
cmake_minimum_required(VERSION 2.8)

project(diagnostics-tool)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

add_executable(
    diagnostics-tool
    src/brake_module_state.c
    src/can_monitor.c
    src/diagnostics.c
    src/steering_module_state.c
    src/system_state.c
    src/terminal_print.c
    src/throttle_module_state.c)

target_include_directories(
    diagnostics-tool
    PRIVATE
    include
    ../../api/include
    ${SDL2_INCLUDE_DIRS})

target_link_libraries(
    diagnostics-tool
    PRIVATE
    ${SDL2_LIBRARIES}
    canlib)
