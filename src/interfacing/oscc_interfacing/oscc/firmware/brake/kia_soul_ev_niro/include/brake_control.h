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
 * @file brake_control.h
 * @brief Control of the brake system.
 *
 */

#ifndef _OSCC_BRAKE_CONTROL_H_
#define _OSCC_BRAKE_CONTROL_H_

#include <stdint.h>

/**
 * @brief Brake pedal position values.
 *
 * Contains high and low brake pedal values.
 *
 */
typedef struct
{
  uint16_t low; /* Low value of brake pedal position. */

  uint16_t high; /* High value of brake pedal position. */
} brake_pedal_position_s;

/**
 * @brief Current brake control state.
 *
 * Current state of the brake module control system.
 *
 */
typedef struct
{
  bool enabled; /* Flag indicating whether control is currently enabled. */

  bool operator_override; /* Flag indicating whether brake pedal was manually
                               pressed by operator. */

  uint8_t dtcs; /* Bitfield of faults present in the module. */
} brake_control_state_s;

// ****************************************************************************
// Function:    check_for_sensor_faults
//
// Purpose:     Checks to see if valid values are being read from the sensors
//              and if the vehicle's operator has manually pressed the brake
//              pedal to disable if they have.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void check_for_faults(void);

// ****************************************************************************
// Function:    update_brake
//
// Purpose:     Writes brake spoof values to DAC.
//
// Returns:     void
//
// Parameters:  spoof_command_high - high value of spoof command
//              spoof_command_low - low value of spoof command
//
// ****************************************************************************
void update_brake(uint16_t spoof_command_high, uint16_t spoof_command_low);

// ****************************************************************************
// Function:    enable_control
//
// Purpose:     Enable control of the brake system.
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
// Purpose:     Disable control of the brake system.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void disable_control(void);

#endif /* _OSCC_BRAKE_CONTROL_H_ */
