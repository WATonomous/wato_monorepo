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
# Ensure that all required paths are found
VALIDATE_VARIABLES_NOT_EMPTY(VARS
        ARDUINO_PLATFORMS
        ARDUINO_CORES_PATH
        ARDUINO_BOOTLOADERS_PATH
        ARDUINO_LIBRARIES_PATH
        ARDUINO_BOARDS_PATH
        ARDUINO_PROGRAMMERS_PATH
        ARDUINO_VERSION_PATH
        ARDUINO_AVRDUDE_FLAGS
        ARDUINO_AVRDUDE_PROGRAM
        ARDUINO_AVRDUDE_CONFIG_PATH
        AVRSIZE_PROGRAM
        ${ADDITIONAL_REQUIRED_VARS}
        MSG "Invalid Arduino SDK path (${ARDUINO_SDK_PATH}).\n")
