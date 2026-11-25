// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Pin encoding shared by the pin map (config/cs_pins.h), the board files and the checks.
// A pin is one byte: port index (A = 0) in the high nibble, pin number in the low nibble.

#ifndef CS_PINS_DEFS_H_
#define CS_PINS_DEFS_H_

#define CS_NO_PIN 0xFF
#define CS_PIN(port, num) ((((port) - 'A') << 4) | (num))
#define CS_PIN_PORT(code) (((code) >> 4) & 0xF)
#define CS_PIN_NUM(code) ((code) & 0xF)
#define CS_PIN_USED(code) ((code) != CS_NO_PIN)

#define CS_PULL_NONE 0
#define CS_PULL_UP 1
#define CS_PULL_DOWN 2

// TIM2 input channel of a pin (alternate function 1 on both STM32F767 and STM32H753), 0 if the
// pin has no TIM2 channel. The PPS input must be one of these.
#define CS_TIM2_CHANNEL(code)                                                              \
  (((code) == CS_PIN('A', 0) || (code) == CS_PIN('A', 5) || (code) == CS_PIN('A', 15)) ? 1 \
   : ((code) == CS_PIN('A', 1) || (code) == CS_PIN('B', 3))                            ? 2 \
   : ((code) == CS_PIN('A', 2) || (code) == CS_PIN('B', 10))                           ? 3 \
   : ((code) == CS_PIN('A', 3) || (code) == CS_PIN('B', 11))                           ? 4 \
                                                                                       : 0)

#endif  // CS_PINS_DEFS_H_
