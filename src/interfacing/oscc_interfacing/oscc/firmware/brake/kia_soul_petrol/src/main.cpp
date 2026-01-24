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
 * @file main.cpp
 *
 */

#include "accumulator.h"
#include "arduino_init.h"
#include "brake_control.h"
#include "communications.h"
#include "debug.h"
#include "init.h"
#include "timers.h"

int main(void)
{
  init_arduino();

  init_communication_interfaces();

  init_globals();

  init_devices();

  init_communication_interfaces();

  start_timers();

  DEBUG_PRINTLN("init complete");

  while (true) {
    check_for_incoming_message();

    accumulator_maintain_pressure();

    check_for_operator_override();

    update_brake();
  }
}
