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
set(FIRMWARE_SIZE_SCRIPT_PATH "${CMAKE_CURRENT_LIST_DIR}/../Extras/CalculateFirmwareSize.cmake")
file(READ ${FIRMWARE_SIZE_SCRIPT_PATH} FIRMWARE_SIZE_SCRIPT)

# Replace placeholders with matching arguments
string(REGEX REPLACE "PLACEHOLDER_1" "${AVRSIZE_PROGRAM}" FIRMWARE_SIZE_SCRIPT "${FIRMWARE_SIZE_SCRIPT}")

# Create the replaced file in the build's cache directory
set(CACHED_FIRMWARE_SCRIPT_PATH ${CMAKE_BINARY_DIR}/CMakeFiles/FirmwareSize.cmake)
file(WRITE ${CACHED_FIRMWARE_SCRIPT_PATH} "${FIRMWARE_SIZE_SCRIPT}")

set(ARDUINO_SIZE_SCRIPT ${CACHED_FIRMWARE_SCRIPT_PATH} CACHE INTERNAL "Arduino Size Script")
