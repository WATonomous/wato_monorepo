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
# Setups all of Arduino's built-in examples categories, listing it by their names
# without the index prefix ('01.Basics' becomes 'Basics').
# This list is saved in a cached variable named 'ARDUINO_EXAMPLE_CATEGORIES'.

file(GLOB EXAMPLE_CATEGORIES RELATIVE ${ARDUINO_EXAMPLES_PATH} ${ARDUINO_EXAMPLES_PATH}/*)
list(SORT EXAMPLE_CATEGORIES)

foreach (CATEGORY ${EXAMPLE_CATEGORIES})
    if (NOT EXAMPLE_CATEGORY_INDEX_LENGTH)
        string(REGEX MATCH "^[0-9]+" CATEGORY_INDEX ${CATEGORY})
        string(LENGTH ${CATEGORY_INDEX} INDEX_LENGTH)
        set(EXAMPLE_CATEGORY_INDEX_LENGTH ${INDEX_LENGTH} CACHE INTERNAL
                "Number of digits preceeding an example's category path")
    endif ()
    string(REGEX MATCH "[^0-9.]+$" PARSED_CATEGORY ${CATEGORY})
    string(TOLOWER ${PARSED_CATEGORY} PARSED_CATEGORY)
    list(APPEND CATEGORIES "${PARSED_CATEGORY}")
endforeach ()

set(ARDUINO_EXAMPLE_CATEGORIES ${CATEGORIES} CACHE INTERNAL
        "List of categories containing the built-in Arduino examples")
