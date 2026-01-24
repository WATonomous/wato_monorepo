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
if (NOT PLATFORM_PATH)
    # Arduino is assumed as the default platform
    set(PLATFORM_PATH ${ARDUINO_SDK_PATH}/hardware/arduino/)
endif ()
string(REGEX REPLACE "/$" "" PLATFORM_PATH ${PLATFORM_PATH})
GET_FILENAME_COMPONENT(VENDOR_ID ${PLATFORM_PATH} NAME)
GET_FILENAME_COMPONENT(BASE_PATH ${PLATFORM_PATH} PATH)

if (NOT PLATFORM_ARCHITECTURE)
    # avr is the default architecture
    set(PLATFORM_ARCHITECTURE "avr")
endif ()

if (CUSTOM_PLATFORM_REGISTRATION_SCRIPT)
    include("${CUSTOM_PLATFORM_REGISTRATION_SCRIPT}")
else ()
    include(RegisterSpecificHardwarePlatform)
endif ()
