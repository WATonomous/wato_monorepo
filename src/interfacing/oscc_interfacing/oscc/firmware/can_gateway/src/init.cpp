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
 * @file init.cpp
 *
 */

#include "init.h"

#include "debug.h"
#include "globals.h"
#include "oscc_can.h"
#include "oscc_serial.h"

void init_globals(void)
{
  memset(&g_display_state, 0, sizeof(g_display_state));
}

void init_communication_interfaces(void)
{
#ifdef DEBUG
  init_serial();
#endif

  DEBUG_PRINTLN("init display");
  init_display();

  DEBUG_PRINT("init OBD CAN - ");
  init_can(g_obd_can);

  DEBUG_PRINT("init Control CAN - ");
  init_can(g_control_can);
}
