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
project(null)

SET(ARDUINO_DEFAULT_PORT ${SERIAL_PORT_NULL})
set(ARDUINO_DEFAULT_BAUDRATE ${SERIAL_BAUD_NULL})

generate_arduino_firmware(
    null
    SRCS
    main.cpp
    ${OSCC_FIRMWARE_ROOT}/common/libs/arduino_init/arduino_init.cpp)

target_include_directories(
    null
    PRIVATE
    ${OSCC_FIRMWARE_ROOT}/common/libs/arduino_init
)
