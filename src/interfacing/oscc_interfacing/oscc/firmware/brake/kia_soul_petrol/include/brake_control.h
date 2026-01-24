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
 * @brief Current brake control state.
 *
 * Current state of the throttle module control system.
 *
 */
typedef struct
{
  bool enabled; /* Flag indicating control is currently enabled. */

  bool operator_override; /* Flag indicating whether steering wheel was
                               manually turned by operator. */

  uint8_t dtcs; /* Bitfield of faults present in the module. */

  float commanded_pedal_position; /* Brake pedal position commanded by
                                       controller. */

  bool startup_pressure_check_error; /* Flag indicating a problem with the actuator. */

  bool startup_pump_motor_check_error; /* Flag indicating a problem with the pump motor. */
} brake_control_state_s;

// ****************************************************************************
// Function:    set_accumulator_solenoid_duty_cycle
//
// Purpose:     Set the PWM that controls the "accumulator" solenoids to the
//              specified value.
//
// Returns:     void
//
// Parameters:  [in] duty_cycle - value to send to the PWM
//
// ****************************************************************************
void set_accumulator_solenoid_duty_cycle(const uint16_t duty_cycle);

// ****************************************************************************
// Function:    set_release_solenoid_duty_cycle
//
// Purpose:     Set the PWM that controls the "release" solenoids to the
//              specified value.
//
// Returns:     void
//
// Parameters:  [in] duty_cycle - value to send to the PWM
//
// ****************************************************************************
void set_release_solenoid_duty_cycle(const uint16_t duty_cycle);

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

// ****************************************************************************
// Function:    check_for_operator_override
//
// Purpose:     Check to see if the vehicle's operator has manually pressed
//              the brake pedal and disable control if they have.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void check_for_operator_override(void);

// ****************************************************************************
// Function:    check_for_sensor_faults
//
// Purpose:     Checks to see if valid values are being read from the sensors.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void check_for_sensor_faults(void);

// ****************************************************************************
// Function:    brake_init
//
// Purpose:     Initialize the brake system.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void brake_init(void);

// ****************************************************************************
// Function:    update_brake
//
// Purpose:     Write brake spoof values to DAC.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void update_brake(void);

#endif /* _OSCC_BRAKE_CONTROL_H_ */
