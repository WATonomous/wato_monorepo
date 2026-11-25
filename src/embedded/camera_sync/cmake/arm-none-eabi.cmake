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

# GNU Arm Embedded toolchain. Finds arm-none-eabi-gcc on PATH, or in ARM_TOOLCHAIN_DIR
# (CMake variable or environment variable), e.g. the STM32CubeIDE / CubeCLT bundled toolchain.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(_cs_hints)
if(DEFINED ARM_TOOLCHAIN_DIR)
  list(APPEND _cs_hints ${ARM_TOOLCHAIN_DIR} ${ARM_TOOLCHAIN_DIR}/bin)
endif()
if(DEFINED ENV{ARM_TOOLCHAIN_DIR})
  list(APPEND _cs_hints $ENV{ARM_TOOLCHAIN_DIR} $ENV{ARM_TOOLCHAIN_DIR}/bin)
endif()

find_program(CMAKE_C_COMPILER arm-none-eabi-gcc HINTS ${_cs_hints} REQUIRED)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy HINTS ${_cs_hints} REQUIRED)
find_program(CMAKE_SIZE arm-none-eabi-size HINTS ${_cs_hints} REQUIRED)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
