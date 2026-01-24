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
project(brake-utils-release-pressure)

generate_arduino_firmware(
    brake-utils-release-pressure
    SKETCH src/release_pressure.ino
    BOARD_CPU atmega2560
    BOARD mega)

target_compile_options(
    brake-utils-release-pressure
    PRIVATE
    "-std=gnu++11"
    "-Os")
