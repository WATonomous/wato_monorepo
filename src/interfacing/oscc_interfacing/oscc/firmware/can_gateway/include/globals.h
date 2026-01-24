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

/**
 * @file globals.h
 * @brief Module globals.
 *
 */

#ifndef _OSCC_KIA_SOUL_CAN_GATEWAY_GLOBALS_H_
#define _OSCC_KIA_SOUL_CAN_GATEWAY_GLOBALS_H_

#include "display.h"
#include "mcp_can.h"
#include "ssd1325.h"

/*
 * @brief Chip select pin of the OBD CAN IC.
 *
 */
#define PIN_OBD_CAN_CHIP_SELECT (9)

/*
 * @brief Chip select pin of the Control CAN IC.
 *
 */
#define PIN_CONTROL_CAN_CHIP_SELECT (10)

/*
 * @brief SPI CS pin to display.
 *
 */
#define PIN_DISPLAY_CHIP_SELECT (8)

#ifdef GLOBAL_DEFINED
MCP_CAN g_obd_can(PIN_OBD_CAN_CHIP_SELECT);
MCP_CAN g_control_can(PIN_CONTROL_CAN_CHIP_SELECT);
  #ifndef TESTS
SSD1325 g_display(PIN_DISPLAY_CHIP_SELECT);
  #endif

  #define EXTERN
#else
extern MCP_CAN g_obd_can;
extern MCP_CAN g_control_can;
extern SSD1325 g_display;

  #define EXTERN extern
#endif

EXTERN kia_soul_gateway_display_state_s g_display_state;

#endif /* _OSCC_KIA_SOUL_CAN_GATEWAY_GLOBALS_H_ */
