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
set(ARDUINO_DEFAULT_BOARD uno CACHE STRING "Default Arduino Board ID when not specified.")
set(ARDUINO_DEFAULT_PORT CACHE STRING "Default Arduino port when not specified.")
set(ARDUINO_DEFAULT_SERIAL CACHE STRING "Default Arduino Serial command when not specified.")
set(ARDUINO_DEFAULT_PROGRAMMER CACHE STRING "Default Arduino Programmer ID when not specified.")
set(ARDUINO_CMAKE_RECURSION_DEFAULT FALSE CACHE BOOL
        "The default recursion behavior during library setup")
