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
# make_arduino_library_example
# [PRIVATE/INTERNAL]
#
# make_arduino_library_example(TARGET_NAME LIBRARY_NAME EXAMPLE_NAME OUTPUT_VAR)
#
#      TARGET_NAME  - Target name
#      LIBRARY_NAME - Library name
#      EXAMPLE_NAME - Example name
#      OUTPUT_VAR   - Variable name to save sketch path.
#
# Creates a Arduino example from the specified library.
#=============================================================================#
function(make_arduino_library_example TARGET_NAME LIBRARY_NAME EXAMPLE_NAME OUTPUT_VAR)
    set(EXAMPLE_SKETCH_PATH)

    get_property(LIBRARY_SEARCH_PATH
            DIRECTORY     # Property Scope
            PROPERTY LINK_DIRECTORIES)
    foreach (LIB_SEARCH_PATH ${LIBRARY_SEARCH_PATH} ${ARDUINO_LIBRARIES_PATH}
            ${ARDUINO_PLATFORM_LIBRARIES_PATH} ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/libraries)
        if (EXISTS "${LIB_SEARCH_PATH}/${LIBRARY_NAME}/examples/${EXAMPLE_NAME}")
            set(EXAMPLE_SKETCH_PATH "${LIB_SEARCH_PATH}/${LIBRARY_NAME}/examples/${EXAMPLE_NAME}")
            break()
        endif ()
    endforeach ()

    if (EXAMPLE_SKETCH_PATH)
        make_arduino_sketch(${TARGET_NAME} ${EXAMPLE_SKETCH_PATH} SKETCH_CPP)
        set("${OUTPUT_VAR}" ${${OUTPUT_VAR}} ${SKETCH_CPP} PARENT_SCOPE)
    else ()
        message(FATAL_ERROR "Could not find example ${EXAMPLE_NAME} from library ${LIBRARY_NAME}")
    endif ()
endfunction()
