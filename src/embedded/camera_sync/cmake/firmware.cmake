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

# Firmware for one board. Everything chip-specific is selected here; the sources in app/ and
# port/stm32/ are the same for both boards.

if(NOT CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR "CS_BOARD=${CS_BOARD} needs the Arm toolchain: use a preset (cmake --preset f767) "
                      "or -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake")
endif()

set(CS_BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/boards/${CS_BOARD})
set(CS_DRIVERS ${CMAKE_CURRENT_SOURCE_DIR}/Drivers)
set(CS_HAL_MODULES core cortex dma dma_ex eth flash flash_ex gpio pwr pwr_ex rcc rcc_ex uart uart_ex)

if(CS_BOARD STREQUAL "nucleo_f767zi")
  set(CS_FAMILY F7)
  set(CS_DEVICE STM32F767xx)
  set(CS_STARTUP ${CS_BOARD_DIR}/startup_stm32f767xx.s)
  set(CS_SYSTEM ${CS_BOARD_DIR}/system_stm32f7xx.c)
  set(CS_LINKER_SCRIPT ${CS_BOARD_DIR}/STM32F767ZITx_FLASH.ld)
  set(CS_HAL_DIR ${CS_DRIVERS}/STM32F7xx_HAL_Driver)
  set(CS_HAL_PREFIX stm32f7xx_hal)
  set(CS_CMSIS_DEVICE ${CS_DRIVERS}/CMSIS/Device/ST/STM32F7xx/Include)
  list(APPEND CS_HAL_MODULES exti)
elseif(CS_BOARD STREQUAL "nucleo_h753zi")
  set(CS_FAMILY H7)
  set(CS_DEVICE STM32H753xx)
  set(CS_STARTUP ${CS_BOARD_DIR}/startup_stm32h753zitx.s)
  set(CS_SYSTEM ${CS_BOARD_DIR}/system_stm32h7xx.c)
  set(CS_LINKER_SCRIPT ${CS_BOARD_DIR}/STM32H753ZITx_FLASH.ld)
  set(CS_HAL_DIR ${CS_DRIVERS}/STM32H7xx_HAL_Driver)
  set(CS_HAL_PREFIX stm32h7xx_hal)
  set(CS_CMSIS_DEVICE ${CS_DRIVERS}/CMSIS/Device/ST/STM32H7xx/Include)
  list(APPEND CS_HAL_MODULES eth_ex exti hsem mdma)
else()
  message(FATAL_ERROR "Unknown CS_BOARD '${CS_BOARD}': use nucleo_f767zi or nucleo_h753zi")
endif()
string(TOUPPER ${CS_BOARD} CS_BOARD_UPPER)

set(CS_CPU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)
set(CS_DEFINES ${CS_DEVICE} USE_HAL_DRIVER CS_FAMILY_${CS_FAMILY} CS_BOARD_${CS_BOARD_UPPER})
set(CS_INCLUDES
  ${CS_APP_INCLUDES}
  ${CMAKE_CURRENT_SOURCE_DIR}/port/stm32
  ${CS_BOARD_DIR}
)
# Vendor headers are system headers: our -Werror flags do not apply to them.
set(CS_VENDOR_INCLUDES
  ${CS_HAL_DIR}/Inc
  ${CS_CMSIS_DEVICE}
  ${CS_DRIVERS}/CMSIS/Include
)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(${CS_CPU_FLAGS} -O2 -g3 -ffunction-sections -fdata-sections)

# Vendor code: ST HAL, CMSIS system file and startup. Built without our warning flags.
set(CS_HAL_SOURCES)
foreach(module ${CS_HAL_MODULES})
  if(module STREQUAL "core")
    list(APPEND CS_HAL_SOURCES ${CS_HAL_DIR}/Src/${CS_HAL_PREFIX}.c)
  else()
    list(APPEND CS_HAL_SOURCES ${CS_HAL_DIR}/Src/${CS_HAL_PREFIX}_${module}.c)
  endif()
endforeach()
add_library(cs_vendor STATIC ${CS_HAL_SOURCES} ${CS_SYSTEM} ${CS_STARTUP})
target_compile_definitions(cs_vendor PUBLIC ${CS_DEFINES})
target_include_directories(cs_vendor PUBLIC ${CS_INCLUDES})
target_include_directories(cs_vendor SYSTEM PUBLIC ${CS_VENDOR_INCLUDES})

# Our code, with warnings as errors.
set(CS_PORT_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/port/stm32/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/port/stm32/pins.c
  ${CMAKE_CURRENT_SOURCE_DIR}/port/stm32/trigger.c
  ${CMAKE_CURRENT_SOURCE_DIR}/port/stm32/uart_tx.c
  ${CMAKE_CURRENT_SOURCE_DIR}/port/stm32/eth_port.c
  ${CMAKE_CURRENT_SOURCE_DIR}/port/stm32/stm32_it.c
  ${CS_BOARD_DIR}/board_clock.c
)
set(CS_ELF camera_sync_${CS_BOARD})
add_executable(${CS_ELF} ${CS_APP_SOURCES} ${CS_PORT_SOURCES})
set_target_properties(${CS_ELF} PROPERTIES SUFFIX ".elf")
target_compile_options(${CS_ELF} PRIVATE ${CS_WARNINGS})
target_link_libraries(${CS_ELF} PRIVATE cs_vendor)
target_link_options(${CS_ELF} PRIVATE
  ${CS_CPU_FLAGS}
  -T${CS_LINKER_SCRIPT}
  -L${CMAKE_CURRENT_SOURCE_DIR}/boards/common
  --specs=nano.specs -nostartfiles  # the CMSIS startup file is the C runtime start: no crt0, exit or stdio
  -Wl,--gc-sections -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${CS_ELF}.map
)
set_target_properties(${CS_ELF} PROPERTIES LINK_DEPENDS
  "${CS_LINKER_SCRIPT};${CMAKE_CURRENT_SOURCE_DIR}/boards/common/sections.ld")

add_custom_command(TARGET ${CS_ELF} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CS_ELF}> ${CMAKE_CURRENT_BINARY_DIR}/${CS_ELF}.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CS_ELF}> ${CMAKE_CURRENT_BINARY_DIR}/${CS_ELF}.hex
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CS_ELF}>
  COMMENT "Writing ${CS_ELF}.bin and ${CS_ELF}.hex"
)
