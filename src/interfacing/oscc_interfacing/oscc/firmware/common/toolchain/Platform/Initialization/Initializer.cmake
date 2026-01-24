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
#                          Initialization
#=============================================================================#

if (NOT ARDUINO_FOUND AND ARDUINO_SDK_PATH)

    include(SetupCompilerSettings)
    include(SetupArduinoSettings)

    # get version first (some stuff depends on versions)
    include(DetectVersion)

    include(RegisterHardwarePlatform)
    include(FindPrograms)
    include(SetDefaults)
    include(SetupFirmwareSizeScript)
    include(SetupLibraryBlacklist)

    include(TestSetup)
    include(DefineAdvancedVariables)

    set(ARDUINO_FOUND True CACHE INTERNAL "Arduino Found")

endif ()
