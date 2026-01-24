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
* @file throttle_module_state.c
* @brief Throttle module Source.
*
*/

#include "throttle_module_state.h"

#include <stdio.h>
#include <stdlib.h>

#include "can_monitor.h"
#include "can_protocols/throttle_can_protocol.h"
#include "macros.h"
#include "system_state.h"

// *****************************************************
// static definitions
// *****************************************************

//
static int analyze_command_frame(
  throttle_module_state_s * const state, const oscc_throttle_command_s * const throttle_command)
{
  int module_state = STATE_OK;

  return module_state;
}

//
static int analyze_report_frame(
  throttle_module_state_s * const state, const oscc_throttle_report_s * const throttle_report)
{
  int module_state = STATE_OK;

  state->control_state = throttle_report->enabled;

  state->override_triggered = throttle_report->operator_override;

  return module_state;
}

// *****************************************************
// public definitions
// *****************************************************

//
int analyze_throttle_state(
  throttle_module_state_s * const state,
  const can_frame_s * const throttle_command_frame,
  const can_frame_s * const throttle_report_frame)
{
  int ret = NOERR;

  analyze_command_frame(
    state,
    (oscc_throttle_command_s *)throttle_command_frame->frame_contents.buffer);  // TODO : do we need this?

  state->module_state =
    analyze_report_frame(state, (oscc_throttle_report_s *)throttle_report_frame->frame_contents.buffer);

  return ret;
}
