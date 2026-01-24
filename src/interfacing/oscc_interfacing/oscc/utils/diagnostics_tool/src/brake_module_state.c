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
* @file brake_module_state.c
* @brief brake module Source.
*
*/

#include "brake_module_state.h"

#include <stdio.h>
#include <stdlib.h>

#include "can_monitor.h"
#include "can_protocols/brake_can_protocol.h"
#include "macros.h"
#include "system_state.h"

// *****************************************************
// static definitions
// *****************************************************

//
static int analyze_command_frame(brake_module_state_s * const state, const oscc_brake_command_s * const brake_command)
{
  int module_state = STATE_OK;

  return module_state;
}

//
static int analyze_report_frame(brake_module_state_s * const state, const oscc_brake_report_s * const brake_report)
{
  int module_state = STATE_OK;

  state->control_state = brake_report->enabled;

  state->override_triggered = brake_report->operator_override;

  if (brake_report->dtcs != 0) {
    module_state = STATE_FAULT;
  }

  return module_state;
}

// *****************************************************
// public definitions
// *****************************************************

//
int analyze_brake_state(
  brake_module_state_s * const state,
  const can_frame_s * const brake_command_frame,
  const can_frame_s * const brake_report_frame)
{
  int ret = NOERR;

  analyze_command_frame(
    state,
    (oscc_brake_command_s *)brake_command_frame->frame_contents.buffer);  // TODO : do we need this?

  state->module_state = analyze_report_frame(state, (oscc_brake_report_s *)brake_report_frame->frame_contents.buffer);

  return ret;
}
