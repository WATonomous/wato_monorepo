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
project(steering)

SET(ARDUINO_DEFAULT_PORT ${SERIAL_PORT_STEERING})
set(ARDUINO_DEFAULT_BAUDRATE ${SERIAL_BAUD_STEERING})

add_definitions(-DSERIAL_BAUD=${ARDUINO_DEFAULT_BAUDRATE})

add_custom_target(
    steering-monitor
    COMMAND screen ${ARDUINO_DEFAULT_PORT} ${ARDUINO_DEFAULT_BAUDRATE})

add_custom_target(
    steering-monitor-log
    COMMAND screen -L ${ARDUINO_DEFAULT_PORT} ${ARDUINO_DEFAULT_BAUDRATE})

generate_arduino_firmware(
    steering
    SRCS
    ${OSCC_FIRMWARE_ROOT}/common/libs/arduino_init/arduino_init.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/fault_check/oscc_check.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/mcp_can/mcp_can.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/pid/oscc_pid.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/serial/oscc_serial.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/can/oscc_can.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/dac/oscc_dac.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/timer/oscc_timer.cpp
    src/main.cpp
    src/globals.cpp
    src/init.cpp
    src/communications.cpp
    src/steering_control.cpp)

target_include_directories(
    steering
    PRIVATE
    include
    ${OSCC_FIRMWARE_ROOT}/common/include
    ${OSCC_FIRMWARE_ROOT}/common/libs/arduino_init
    ${OSCC_FIRMWARE_ROOT}/common/libs/DAC_MCP49xx
    ${OSCC_FIRMWARE_ROOT}/common/libs/fault_check
    ${OSCC_FIRMWARE_ROOT}/common/libs/mcp_can
    ${OSCC_FIRMWARE_ROOT}/common/libs/pid
    ${OSCC_FIRMWARE_ROOT}/common/libs/serial
    ${OSCC_FIRMWARE_ROOT}/common/libs/can
    ${OSCC_FIRMWARE_ROOT}/common/libs/dac
    ${OSCC_FIRMWARE_ROOT}/common/libs/timer
    ${OSCC_FIRMWARE_ROOT}/../api/include)
