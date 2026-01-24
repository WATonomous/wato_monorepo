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
* @file throttle_module_state.h
* @brief Throttle module state.
*
**/

#ifndef THROTTLE_MODULE_STATE_H
#define THROTTLE_MODULE_STATE_H

#include <stdint.h>

/**
 * @brief Throttle module state information.
 *
 * Contains state information for the throttle module.
 *
 */
typedef struct
{
  //
  //
  uint8_t module_state;
  //
  //
  uint8_t control_state;
  //
  //
  uint8_t override_triggered;
} throttle_module_state_s;

int analyze_throttle_state(
  throttle_module_state_s * const state,
  const can_frame_s * const throttle_command_frame,
  const can_frame_s * const throttle_report_frame);

#endif /* THROTTLE_MODULE_STATE_H */
