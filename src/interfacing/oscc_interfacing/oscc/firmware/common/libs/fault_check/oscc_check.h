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
 * @file oscc_check.h
 * @brief common fault checking functions.
 *
 */

#ifndef _OSCC_CHECK_H_
#define _OSCC_CHECK_H_

#include <stdint.h>

typedef struct
{
  //! Flag set if a fault condition has been detected and is being monitored.
  bool monitoring_active;

  //! The time (in ms) when the fault condition was first detected.
  unsigned long condition_start_time;
} condition_state_s;

//! Initialize a condition state object to safe defaults.
#define CONDITION_STATE_INIT {.monitoring_active = false, .condition_start_time = 0}

// ****************************************************************************
// Function:    condition_exceeded_duration
//
// Purpose:     Check if a fault condition has been active for longer than
//              `max_duration`.
//
// Returns:     bool where true means a fault condition has been active for
//              longer than `max_duration`.
//
// Parameters:  [in] condition_active - true if a fault condition has been
//                                      detected.
//              [in] max_duration - the maximum time that the fault condition
//                                  can be active before being reported as an
//                                  actual fault.
//              [in] condition_state - whether the fault condition is active
//                                     and at what time the condition was first
//                                     detected.
// ****************************************************************************
bool condition_exceeded_duration(bool condition_active, unsigned long max_duration, condition_state_s * state);

// ****************************************************************************
// Function:    check_voltage_grounded
//
// Purpose:     Check if the voltage is grounded for the period of time
//              determined by `max_duration`. This function is assumed to
//              be called periodicly.
//
// Returns:     bool where true means a fault condition has been active for
//              longer than `max_duration`.
//
// Parameters:  [in] high - the high pin to check for ground
//              [in] low - the low pin to check for ground
//              [in] max_duration - the maximum time that the fault condition
//                                  can be active before being reported as an
//                                  actual fault.
//              [in] condition_state - whether the fault condition is active
//                                     and at what time the condition was first
//                                     detected.
//
// ****************************************************************************
bool check_voltage_grounded(
  uint16_t high, uint16_t low, unsigned long max_duration, condition_state_s * condition_state);

#endif /* _OSCC_CHECK_H_ */
