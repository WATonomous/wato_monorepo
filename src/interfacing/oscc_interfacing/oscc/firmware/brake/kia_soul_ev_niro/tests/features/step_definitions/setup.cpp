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

#include "brake_control.h"
#include "can_protocols/brake_can_protocol.h"
#include "can_protocols/fault_can_protocol.h"
#include "communications.h"
#include "globals.h"

/** Define the module name under test
 *
 * In order to implement reusable test fixtures and steps, those fixtures
 * need the name of the module under test defined.
 *
 * \sa firmware/common/testing/step_definitions/common.cpp
 */
#define FIRMWARE_NAME "brake"

/** Define aliases to the brake control state
 *
 * \sa firmware/common/testing/step_definitions/fault_checking.cpp
 */
#define g_vcm_control_state g_brake_control_state

extern volatile brake_control_state_s g_brake_control_state;

/** Define the origin ID that brake faults should be associated with
 *
 * \sa firmware/common/testing/step_definitions/fault_checking.cpp
 */
const int MODULE_FAULT_ORIGIN_ID = FAULT_ORIGIN_BRAKE;
