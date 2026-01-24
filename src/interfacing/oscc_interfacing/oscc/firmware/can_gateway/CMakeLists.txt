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
project(can-gateway)

SET(ARDUINO_DEFAULT_PORT ${SERIAL_PORT_CAN_GATEWAY})
set(ARDUINO_DEFAULT_BAUDRATE ${SERIAL_BAUD_CAN_GATEWAY})

add_definitions(-DSERIAL_BAUD=${ARDUINO_DEFAULT_BAUDRATE})

add_custom_target(
    can-gateway-monitor
    COMMAND screen ${ARDUINO_DEFAULT_PORT} ${ARDUINO_DEFAULT_BAUDRATE})

add_custom_target(
    can-gateway-monitor-log
    COMMAND screen -L ${ARDUINO_DEFAULT_PORT} ${ARDUINO_DEFAULT_BAUDRATE})

generate_arduino_firmware(
    can-gateway
    SRCS
    ${OSCC_FIRMWARE_ROOT}/common/libs/arduino_init/arduino_init.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/mcp_can/mcp_can.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/serial/oscc_serial.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/can/oscc_can.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/timer/oscc_timer.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/ssd1325/ssd1325.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/ssd1325/gfx/gfx.cpp
    src/main.cpp
    src/globals.cpp
    src/init.cpp
    src/communications.cpp
    src/display.cpp
    src/timer.cpp)

target_include_directories(
    can-gateway
    PRIVATE
    include
    ${OSCC_FIRMWARE_ROOT}/common/include
    ${OSCC_FIRMWARE_ROOT}/common/libs/arduino_init
    ${OSCC_FIRMWARE_ROOT}/common/libs/mcp_can
    ${OSCC_FIRMWARE_ROOT}/common/libs/serial
    ${OSCC_FIRMWARE_ROOT}/common/libs/can
    ${OSCC_FIRMWARE_ROOT}/common/libs/timer
    ${OSCC_FIRMWARE_ROOT}/common/libs/ssd1325
    ${OSCC_FIRMWARE_ROOT}/common/libs/ssd1325/gfx
    ${OSCC_FIRMWARE_ROOT}/../api/include)
