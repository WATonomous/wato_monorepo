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
#=============================================================================#
# build_arduino_bootloader_arguments
# [PRIVATE/INTERNAL]
#
# build_arduino_bootloader_arguments(BOARD_ID TARGET_NAME PORT AVRDUDE_FLAGS OUTPUT_VAR)
#
#      BOARD_ID    - board id
#      TARGET_NAME - target name
#      PORT        - serial port
#      AVRDUDE_FLAGS - avrdude flags (override)
#      OUTPUT_VAR  - name of output variable for result
#
# Sets up default avrdude settings for uploading firmware via the bootloader.
#=============================================================================#
function(build_arduino_bootloader_arguments BOARD_ID TARGET_NAME PORT AVRDUDE_FLAGS OUTPUT_VAR)
    set(AVRDUDE_ARGS ${${OUTPUT_VAR}})

    if (NOT AVRDUDE_FLAGS)
        set(AVRDUDE_FLAGS ${ARDUINO_AVRDUDE_FLAGS})
    endif ()
    _get_board_property(${BOARD_ID} build.mcu MCU)
    list(APPEND AVRDUDE_ARGS
            "-C${ARDUINO_AVRDUDE_CONFIG_PATH}"  # avrdude config
            "-p${MCU}"        # MCU Type
            )

    # Programmer
    _get_board_property(${BOARD_ID} upload.protocol UPLOAD_PROTOCOL)
    if (${UPLOAD_PROTOCOL} STREQUAL "stk500")
        list(APPEND AVRDUDE_ARGS "-cstk500v1")
    else ()
        list(APPEND AVRDUDE_ARGS "-c${UPLOAD_PROTOCOL}")
    endif ()


    _get_board_property(${BOARD_ID} upload.speed UPLOAD_SPEED)
    list(APPEND AVRDUDE_ARGS
            "-b${UPLOAD_SPEED}"     # Baud rate
            "-P${PORT}"             # Serial port
            "-D"                    # Dont erase
            )

    list(APPEND AVRDUDE_ARGS ${AVRDUDE_FLAGS})

    set(${OUTPUT_VAR} ${AVRDUDE_ARGS} PARENT_SCOPE)

endfunction()
