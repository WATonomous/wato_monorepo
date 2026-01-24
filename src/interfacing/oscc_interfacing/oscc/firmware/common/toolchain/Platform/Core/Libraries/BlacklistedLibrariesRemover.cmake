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
# _REMOVE_BLACKLISTED_LIBRARIES
# [PRIVATE/INTERNAL]
#
# _REMOVE_BLACKLISTED_LIBRARIES(LIBRARY_LIST OUTPUT_VAR)
#
#      LIBRARY_LIST - List of libraries to remove blacklisted libraries from
#      OUTPUT_VAR - Original list of libraries without blacklisted libraries.
#
#      Returns a list of libraries without blacklisted libraries.
#
# Arduino-CMake looks for Arduino libraries whose names match those of include files encountered.
# Under certain circumstances this leads to undesired behavior.
# An example is an include file 'Keyboard.h' in a project that has no relation to
# Arduino's standard Keyboard library.
# To prevent the standard library from being falsely added to the list of libraries to build,
# this function removes all blacklisted libraries from the source list, which contains
# all the included libraries found in the search process.
# In the example above the blacklisted and remove library would be
# ${ARDUINO_SDK_PATH}/libraries/Keyboard/src).
#
function(_REMOVE_BLACKLISTED_LIBRARIES LIBRARY_LIST OUTPUT_VAR)
    set(NEW_LIBS)
    foreach (LIB ${LIBRARY_LIST})
        list(FIND ARDUINO_LIBRARY_BLACKLIST "${LIB}" BLACKLISTED_LIB_INDEX)
        if (NOT ${BLACKLISTED_LIB_INDEX} GREATER -1)
            list(APPEND NEW_LIBS "${LIB}")
        else ()
            ARDUINO_DEBUG_MSG("Suppressing blacklisted library ${LIB}")
        endif ()
    endforeach ()

    set("${OUTPUT_VAR}" "${NEW_LIBS}" PARENT_SCOPE)
endfunction()
