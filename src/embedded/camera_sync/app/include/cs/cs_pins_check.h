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

// Compile-time checks of the pin map, and the derived constants the port uses.
//
// Included by the firmware (port/stm32/pins.c) and by a host test, so a bad pin map fails every
// build with a message naming the problem and the port.
//
// How duplicates are found without comparing every pair: for one port, the sum of (1 << pin) over
// all roles equals the OR of the same terms exactly when no pin appears twice.

#ifndef CS_PINS_CHECK_H_
#define CS_PINS_CHECK_H_

#include <stdint.h>

#include "board.h"
#include "cs/cs_pins_defs.h"
#ifdef CS_PINS_HEADER
  #include CS_PINS_HEADER  // host tests swap in bad pin maps
#else
  #include "cs_pins.h"
#endif

// ---------------------------------------------------------------------------------------------
// Role lists

#define CS__TRIG_NUMS(X, a) \
  X(CS_TRIG_1, a)           \
  X(CS_TRIG_2, a)           \
  X(CS_TRIG_3, a)           \
  X(CS_TRIG_4, a)           \
  X(CS_TRIG_5, a)           \
  X(CS_TRIG_6, a)           \
  X(CS_TRIG_7, a)           \
  X(CS_TRIG_8, a)           \
  X(CS_TRIG_9, a)           \
  X(CS_TRIG_10, a)          \
  X(CS_TRIG_11, a)          \
  X(CS_TRIG_12, a)

#define CS__TRIG_CODE(n) (CS_PIN_USED(n) ? CS_PIN(CS_TRIG_PORT, ((n) & 0xF)) : CS_NO_PIN)

#define CS__TRIG_PINS(X, a)       \
  X(CS__TRIG_CODE(CS_TRIG_1), a)  \
  X(CS__TRIG_CODE(CS_TRIG_2), a)  \
  X(CS__TRIG_CODE(CS_TRIG_3), a)  \
  X(CS__TRIG_CODE(CS_TRIG_4), a)  \
  X(CS__TRIG_CODE(CS_TRIG_5), a)  \
  X(CS__TRIG_CODE(CS_TRIG_6), a)  \
  X(CS__TRIG_CODE(CS_TRIG_7), a)  \
  X(CS__TRIG_CODE(CS_TRIG_8), a)  \
  X(CS__TRIG_CODE(CS_TRIG_9), a)  \
  X(CS__TRIG_CODE(CS_TRIG_10), a) \
  X(CS__TRIG_CODE(CS_TRIG_11), a) \
  X(CS__TRIG_CODE(CS_TRIG_12), a)

#define CS__STROBE_PINS(X, a) \
  X(CS_STROBE_1, a)           \
  X(CS_STROBE_2, a)           \
  X(CS_STROBE_3, a)           \
  X(CS_STROBE_4, a)           \
  X(CS_STROBE_5, a)           \
  X(CS_STROBE_6, a)           \
  X(CS_STROBE_7, a)           \
  X(CS_STROBE_8, a)           \
  X(CS_STROBE_9, a)           \
  X(CS_STROBE_10, a)          \
  X(CS_STROBE_11, a)          \
  X(CS_STROBE_12, a)

#define CS__LED_PINS(X, a) X(CS_LED_LOCK, a) X(CS_LED_LINK, a) X(CS_LED_FAULT, a)

#define CS__USED_PINS(X, a) CS__TRIG_PINS(X, a) CS__STROBE_PINS(X, a) X(CS_PPS_PIN, a) CS__LED_PINS(X, a)

// ---------------------------------------------------------------------------------------------
// Term builders

#define CS__BIT(code) (1ull << CS_PIN_NUM(code))
#define CS__ADD_ON_PORT(code, p) +((CS_PIN_USED(code) && CS_PIN_PORT(code) == (p)) ? CS__BIT(code) : 0ull)
#define CS__OR_ON_PORT(code, p) | ((CS_PIN_USED(code) && CS_PIN_PORT(code) == (p)) ? CS__BIT(code) : 0ull)
#define CS__ADD_NUM(code, a) +(CS_PIN_USED(code) ? CS__BIT(code) : 0ull)
#define CS__OR_NUM(code, a) | (CS_PIN_USED(code) ? CS__BIT(code) : 0ull)
#define CS__COUNT(code, a) +(CS_PIN_USED(code) ? 1 : 0)
#define CS__BAD_TRIG_NUM(n, a) +((CS_PIN_USED(n) && ((n) < 0 || (n) > 15)) ? 1 : 0)
#define CS__TRIG_BIT(n, a) | (CS_PIN_USED(n) ? (1u << ((n) & 0xF)) : 0u)

#define CS__PORT_SUM(p) (0ull CS__USED_PINS(CS__ADD_ON_PORT, p))
#define CS__PORT_OR(p) (0ull CS__USED_PINS(CS__OR_ON_PORT, p))
#define CS__PORT_RESERVED(p) (0ull CS_BOARD_RESERVED(CS__OR_ON_PORT, p))

// ---------------------------------------------------------------------------------------------
// Checks

#define CS__CHECK_PORT(p, letter)                                                         \
  _Static_assert(                                                                         \
    CS__PORT_SUM(p) == CS__PORT_OR(p),                                                    \
    "cs_pins.h: two roles (trigger, strobe, PPS, LED) use the same pin on port " letter); \
  _Static_assert(                                                                         \
    (CS__PORT_OR(p) & CS__PORT_RESERVED(p)) == 0ull,                                      \
    "cs_pins.h: a pin on port " letter                                                    \
    " is reserved by the board (Ethernet, ST-LINK UART, SWD, "                            \
    "oscillator, USB), see boards/<board>/board.h")

CS__CHECK_PORT(0, "A");
CS__CHECK_PORT(1, "B");
CS__CHECK_PORT(2, "C");
CS__CHECK_PORT(3, "D");
CS__CHECK_PORT(4, "E");
CS__CHECK_PORT(5, "F");
CS__CHECK_PORT(6, "G");
CS__CHECK_PORT(7, "H");
CS__CHECK_PORT(8, "I");
CS__CHECK_PORT(9, "J");
CS__CHECK_PORT(10, "K");

_Static_assert(CS_TRIG_PORT >= 'A' && CS_TRIG_PORT <= 'K', "cs_pins.h: CS_TRIG_PORT must be a port letter 'A'..'K'");
_Static_assert((0 CS__TRIG_NUMS(CS__BAD_TRIG_NUM, ~)) == 0, "cs_pins.h: CS_TRIG_n must be a pin number 0..15");
_Static_assert((0 CS__TRIG_NUMS(CS__COUNT, ~)) > 0, "cs_pins.h: define at least one trigger output");
_Static_assert(
  (0ull CS__STROBE_PINS(CS__ADD_NUM, ~)) == (0ull CS__STROBE_PINS(CS__OR_NUM, ~)),
  "cs_pins.h: two strobe inputs use the same pin number; each pin number has one EXTI line");
_Static_assert(
  CS_PIN_USED(CS_PPS_PIN) && CS_TIM2_CHANNEL(CS_PPS_PIN) != 0,
  "cs_pins.h: CS_PPS_PIN must be a TIM2 channel pin (PA0, PA5, PA15, PB3, PB10, PA3, PB11)");

// ---------------------------------------------------------------------------------------------
// Derived constants

// Bit n set = pin n of CS_TRIG_PORT is a trigger output.
#define CS_TRIG_PIN_MASK ((uint16_t)(0u CS__TRIG_NUMS(CS__TRIG_BIT, ~)))

// Strobe inputs, index k = bit k of strobe_mask (CS_NO_PIN if unused).
#define CS_STROBE_PIN_LIST \
  {CS_STROBE_1,            \
   CS_STROBE_2,            \
   CS_STROBE_3,            \
   CS_STROBE_4,            \
   CS_STROBE_5,            \
   CS_STROBE_6,            \
   CS_STROBE_7,            \
   CS_STROBE_8,            \
   CS_STROBE_9,            \
   CS_STROBE_10,           \
   CS_STROBE_11,           \
   CS_STROBE_12}
#define CS_STROBE_COUNT 12
#define CS_STROBE_USED_COUNT (0 CS__STROBE_PINS(CS__COUNT, ~))

#define CS_PPS_TIM2_CHANNEL CS_TIM2_CHANNEL(CS_PPS_PIN)

#endif  // CS_PINS_CHECK_H_
