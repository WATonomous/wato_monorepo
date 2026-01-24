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
 * @file throttle_control.h
 * @brief Control of the throttle system.
 *
 */

#ifndef _OSCC_THROTTLE_CONTROL_H_
#define _OSCC_THROTTLE_CONTROL_H_

#include <stdint.h>

/**
 * @brief Accelerator position values.
 *
 * Contains high and low accelerator values.
 *
 */
typedef struct
{
  uint16_t low; /* Low value of accelerator position. */

  uint16_t high; /* High value of accelerator position. */
} accelerator_position_s;

/**
 * @brief Current throttle control state.
 *
 * Current state of the throttle module control system.
 *
 */
typedef struct
{
  bool enabled; /* Flag indicating whether control is currently enabled. */

  bool operator_override; /* Flag indicating whether accelerator was manually
                               pressed by operator. */

  uint8_t dtcs; /* Bitfield of faults present in the module. */
} throttle_control_state_s;

// ****************************************************************************
// Function:    check_for_sensor_faults
//
// Purpose:     Checks to see if valid values are being read from the sensors
//              and if the vehicle's operator has manually pressed the
//              accelerator to disable control if they have.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void check_for_faults(void);

// ****************************************************************************
// Function:    update_throttle
//
// Purpose:     Writes throttle spoof values to DAC.
//
// Returns:     void
//
// Parameters:  spoof_command_high - high value of spoof command
//              spoof_command_low - low value of spoof command
//
// ****************************************************************************
void update_throttle(uint16_t spoof_command_high, uint16_t spoof_command_low);

// ****************************************************************************
// Function:    enable_control
//
// Purpose:     Enable control of the throttle system.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void enable_control(void);

// ****************************************************************************
// Function:    disable_control
//
// Purpose:     Disable control of the throttle system.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void disable_control(void);

#endif /* _OSCC_THROTTLE_CONTROL_H_ */
