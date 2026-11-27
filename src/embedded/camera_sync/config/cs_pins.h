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

// Pin map: the only file to edit when the trigger board schematic changes.
//
// The same map is used for the NUCLEO-F767ZI and the NUCLEO-H753ZI (both NUCLEO-144 boards
// with the same header pinout). Board-specific pins (LEDs, Ethernet, ST-LINK UART) are in
// boards/<board>/board.h. Use `#if defined(CS_BOARD_NUCLEO_H753ZI)` here for a pin that has to
// differ between the two boards.
//
// The build checks this map (app/include/cs/cs_pins_check.h) and fails with a message if:
//   - two roles use the same pin, or a pin is reserved by the board (Ethernet RMII, ST-LINK
//     UART, SWD, oscillator, LEDs);
//   - two strobe inputs share a pin number (each pin number has one EXTI line);
//   - the PPS pin is not a TIM2 channel pin;
//   - no trigger output is defined.
//
// Write pins as CS_PIN('E', 2) for PE2. CS_NO_PIN leaves a role unused.

#ifndef CS_PINS_H_
#define CS_PINS_H_

#include "cs/cs_pins_defs.h"

// ---------------------------------------------------------------------------------------------
// Trigger outputs CAM_TRIG_1..12 (harness labels). All on ONE GPIO port: a single BSRR write
// switches them, so every edge happens at the same instant (issue #561). Give the pin numbers on
// CS_TRIG_PORT. They drive the output driver board, never a camera directly.
#define CS_TRIG_PORT 'E'
#define CS_TRIG_1 2
#define CS_TRIG_2 3
#define CS_TRIG_3 4
#define CS_TRIG_4 5
#define CS_TRIG_5 6
#define CS_TRIG_6 7
#define CS_TRIG_7 8
#define CS_TRIG_8 9
#define CS_TRIG_9 10
#define CS_TRIG_10 11
#define CS_TRIG_11 12
#define CS_TRIG_12 13
#define CS_TRIG_ACTIVE_HIGH 1  // 0: outputs idle high and pulse low

// ---------------------------------------------------------------------------------------------
// Strobe inputs STB1..12 (camera exposure-active outputs). STB(k) is bit k-1 of strobe_mask in
// the TRG line. Any port, but no two inputs with the same pin number.
//
// Not PC0-PC11: PC1, PC4 and PC5 are the Ethernet RMII pins on both NUCLEO boards.
#define CS_STROBE_1 CS_PIN('F', 0)
#define CS_STROBE_2 CS_PIN('F', 1)
#define CS_STROBE_3 CS_PIN('F', 2)
#define CS_STROBE_4 CS_PIN('F', 3)
#define CS_STROBE_5 CS_PIN('F', 4)
#define CS_STROBE_6 CS_PIN('F', 5)
#define CS_STROBE_7 CS_PIN('F', 6)
#define CS_STROBE_8 CS_PIN('F', 7)
#define CS_STROBE_9 CS_PIN('F', 8)
#define CS_STROBE_10 CS_PIN('F', 9)
#define CS_STROBE_11 CS_PIN('F', 10)
#define CS_STROBE_12 CS_PIN('F', 11)
#define CS_STROBE_RISING_EDGE 1  // 0: count falling edges
#define CS_STROBE_PULL CS_PULL_DOWN  // keeps unwired inputs from reporting stray strobes

// ---------------------------------------------------------------------------------------------
// PPS input from the PwrPak7, captured by TIM2 in hardware (no interrupt latency in the time
// stamp). Must be a TIM2 channel pin: PA0, PA5, PA15 (CH1), PB3 (CH2), PB10 (CH3), PA3, PB11
// (CH4). PA1 and PA2 also have TIM2 channels but are Ethernet pins on the NUCLEO boards.
#define CS_PPS_PIN CS_PIN('A', 0)
#define CS_PPS_RISING_EDGE 1  // must match the receiver's PPS polarity (PPSCONTROL ... POSITIVE)

#endif  // CS_PINS_H_
