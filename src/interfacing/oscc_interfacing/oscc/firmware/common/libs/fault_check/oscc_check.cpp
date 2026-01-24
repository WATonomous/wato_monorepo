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
 * @file oscc_check.cpp
 *
 */

#include "oscc_check.h"

#include <Arduino.h>

#include "vehicles.h"

bool condition_exceeded_duration(bool condition_active, unsigned long max_duration, condition_state_s * state)
{
  bool faulted = false;

  if (condition_active == false) {
    /*
         * If a fault condition is not active, update the state to clear
         * the condition active flag and reset the last detection time.
         */
    state->monitoring_active = false;
    state->condition_start_time = 0;
  } else {
    unsigned long now = millis();

    if (state->monitoring_active == false) {
      /* We just detected a condition that may lead to a fault. Update
             * the state to track that the condition is active and store the
             * first time of detection.
             */
      state->monitoring_active = true;
      state->condition_start_time = now;
    }

    unsigned long duration = now - state->condition_start_time;

    if (duration >= max_duration) {
      /* The fault condition has been active for longer than the maximum
             * acceptable duration.
             */
      faulted = true;
    }
  }

  return faulted;
}

bool check_voltage_grounded(uint16_t high, uint16_t low, unsigned long max_duration, condition_state_s * state)
{
  bool condition_active = (high == 0) || (low == 0);
  return condition_exceeded_duration(condition_active, max_duration, state);
}
