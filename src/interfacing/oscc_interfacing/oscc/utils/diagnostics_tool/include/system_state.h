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
* @file system_state.h
* @brief System state.
*
**/

#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>

#include "brake_module_state.h"
#include "gateway_module_state.h"
#include "steering_module_state.h"
#include "throttle_module_state.h"

// Module enabled.
#define CONTROL_ENABLED (1)

// Module disabled.
#define CONTROL_DISABLED (0)

// Module in an OK state
#define STATE_OK (2)

// Module in an WARN state
#define STATE_WARN (1)

// Module in an ERROR state
#define STATE_ERROR (0)

// Module in a non expected state.
#define STATE_FAULT (-1)

/**
 * @brief System state information.
 *
 * Contains system state information for whole system.
 *
 */
typedef struct
{
  //
  //
  throttle_module_state_s throttle_module_state;
  //
  //
  steering_module_state_s steering_module_state;
  //
  //
  brake_module_state_s brake_module_state;
  //
  //
  gateway_module_state_s gateway_module_state;
  //
  //
  uint8_t overall_system_state;
  //
  //
  uint8_t overall_system_control_state;
} current_system_state_s;

//
int update_system_state();

//
void print_system_state();

#endif /* SYSTEM_STATE_H */
