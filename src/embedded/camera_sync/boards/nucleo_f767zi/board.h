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

// NUCLEO-F767ZI (MB1137): pins fixed by the board. Plain macros only (also used by host checks).

#ifndef BOARD_H_
#define BOARD_H_

#include "cs/cs_pins_defs.h"

#define CS_BOARD_NAME "f767zi"

// User LEDs (on = high)
#define CS_LED_LOCK CS_PIN('B', 0)  // LD1 green
#define CS_LED_LINK CS_PIN('B', 7)  // LD2 blue
#define CS_LED_FAULT CS_PIN('B', 14)  // LD3 red

// ST-LINK virtual COM port: USART3, alternate function 7
#define CS_UART_TX_PIN CS_PIN('D', 8)
#define CS_UART_RX_PIN CS_PIN('D', 9)
#define CS_UART_AF 7

// Ethernet: LAN8742A PHY over RMII, alternate function 11
#define CS_ETH_PHY_ADDR 0
#define CS_ETH_AF 11
#define CS_ETH_PINS(X, a)            \
  X(CS_PIN('A', 1), a) /* REF_CLK */ \
  X(CS_PIN('A', 2), a) /* MDIO */    \
  X(CS_PIN('A', 7), a) /* CRS_DV */  \
  X(CS_PIN('C', 1), a) /* MDC */     \
  X(CS_PIN('C', 4), a) /* RXD0 */    \
  X(CS_PIN('C', 5), a) /* RXD1 */    \
  X(CS_PIN('G', 11), a) /* TX_EN */  \
  X(CS_PIN('G', 13), a) /* TXD0 */   \
  X(CS_PIN('B', 13), a) /* TXD1 */

// Pins the pin map must not use.
#define CS_BOARD_RESERVED(X, a)                   \
  CS_ETH_PINS(X, a)                               \
  X(CS_UART_TX_PIN, a)                            \
  X(CS_UART_RX_PIN, a)                            \
  X(CS_PIN('A', 13), a) /* SWDIO */               \
  X(CS_PIN('A', 14), a) /* SWCLK */               \
  X(CS_PIN('H', 0), a) /* HSE in (ST-LINK MCO) */ \
  X(CS_PIN('H', 1), a)                            \
  X(CS_PIN('C', 14), a) /* LSE */                 \
  X(CS_PIN('C', 15), a)                           \
  X(CS_PIN('A', 11), a) /* USB OTG FS */          \
  X(CS_PIN('A', 12), a)                           \
  X(CS_PIN('G', 6), a) /* USB power switch */     \
  X(CS_PIN('G', 7), a) /* USB over-current */

#endif  // BOARD_H_
