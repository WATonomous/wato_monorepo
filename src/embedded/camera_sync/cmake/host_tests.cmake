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

# Host unit tests for the portable app layer and the pin map checks.

enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(cs_app STATIC ${CS_APP_SOURCES})
target_include_directories(cs_app PUBLIC ${CS_APP_INCLUDES})
target_compile_options(cs_app PRIVATE ${CS_WARNINGS})

foreach(name report sched net ptp)
  add_executable(test_${name} tests/test_${name}.c)
  target_link_libraries(test_${name} PRIVATE cs_app m)
  target_compile_options(test_${name} PRIVATE ${CS_WARNINGS} -Wno-double-promotion)
  add_test(NAME ${name} COMMAND test_${name})
endforeach()

# Pin map: the default map must compile for both boards...
set(CS_BOARDS nucleo_f767zi nucleo_h753zi)
foreach(board ${CS_BOARDS})
  add_executable(test_pins_${board} tests/test_pins.c)
  target_include_directories(test_pins_${board} PRIVATE ${CS_APP_INCLUDES} boards/${board})
  target_compile_options(test_pins_${board} PRIVATE ${CS_WARNINGS})
  add_test(NAME pins_${board} COMMAND test_pins_${board})
endforeach()

# ...and every bad map must fail to compile with the matching message.
set(CS_BAD_MAPS
  "strobe_on_trigger|two roles .* on port E"
  "strobe_on_ethernet|port C is reserved by the board"
  "strobe_on_led|two roles .* on port B"
  "strobe_same_number|two strobe inputs use the same pin number"
  "pps_not_tim2|CS_PPS_PIN must be a TIM2 channel pin"
  "no_trigger|define at least one trigger output"
)
foreach(entry ${CS_BAD_MAPS})
  string(REPLACE "|" ";" parts "${entry}")
  list(GET parts 0 map)
  list(GET parts 1 message)
  foreach(board ${CS_BOARDS})
    add_test(
      NAME pins_bad_${map}_${board}
      COMMAND ${CMAKE_C_COMPILER} -std=c11 -fsyntax-only
        -I${CMAKE_CURRENT_SOURCE_DIR}/app/include -I${CMAKE_CURRENT_SOURCE_DIR}/config
        -I${CMAKE_CURRENT_SOURCE_DIR}/boards/${board} -I${CMAKE_CURRENT_SOURCE_DIR}/tests
        "-DCS_PINS_HEADER=\"pins_bad/${map}.h\"" ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pins.c
    )
    set_tests_properties(pins_bad_${map}_${board} PROPERTIES PASS_REGULAR_EXPRESSION "${message}")
  endforeach()
endforeach()

# A map with unused roles and another TIM2 pin is valid.
foreach(board ${CS_BOARDS})
  add_test(
    NAME pins_ok_unused_roles_${board}
    COMMAND ${CMAKE_C_COMPILER} -std=c11 -fsyntax-only -Werror
      -I${CMAKE_CURRENT_SOURCE_DIR}/app/include -I${CMAKE_CURRENT_SOURCE_DIR}/config
      -I${CMAKE_CURRENT_SOURCE_DIR}/boards/${board} -I${CMAKE_CURRENT_SOURCE_DIR}/tests
      "-DCS_PINS_HEADER=\"pins_bad/unused_roles_ok.h\"" ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pins.c
  )
endforeach()
