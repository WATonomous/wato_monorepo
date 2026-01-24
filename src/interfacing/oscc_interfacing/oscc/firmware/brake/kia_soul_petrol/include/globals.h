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
 * @file globals.h
 * @brief Module globals.
 *
 */

#ifndef _OSCC_BRAKE_GLOBALS_H_
#define _OSCC_BRAKE_GLOBALS_H_

#include "brake_control.h"
#include "mcp_can.h"
#include "oscc_pid.h"

/*
 * @brief Chip select pin of the CAN IC.
 *
 */
#define PIN_CAN_CHIP_SELECT (53)

/*
 * @brief Pin of the front left accumulator solenoid.
 *
 * PWM duty cycle controlled by timer 3 (TCCR3B).
 *
 */
#define PIN_ACCUMULATOR_SOLENOID_FRONT_LEFT (5)

/*
 * @brief Pin of the front right accumulator solenoid.
 *
 * PWM duty cycle controlled by timer 4 (TCCR4B).
 *
 */
#define PIN_ACCUMULATOR_SOLENOID_FRONT_RIGHT (7)

/*
 * @brief Pin of the accumulator pressure sensor.
 *
 */
#define PIN_ACCUMULATOR_PRESSURE_SENSOR (9)

/*
 * @brief Pin of the accumulator pump motor.
 *
 */
#define PIN_ACCUMULATOR_PUMP_MOTOR (49)

/*
 * @brief Pin of the accumulator pump motor check (MTT) signal.
 *
 */
#define PIN_ACCUMULATOR_PUMP_MOTOR_CHECK (8)

/*
 * @brief Pin of the master cylinder solenoid.
 *
 */
#define PIN_MASTER_CYLINDER_SOLENOID (2)

/*
 * @brief Pin of the first master cylinder pressure sensor.
 *
 */
#define PIN_MASTER_CYLINDER_PRESSURE_SENSOR_1 (10)

/*
 * @brief Pin of the second master cylinder pressure sensor.
 *
 */
#define PIN_MASTER_CYLINDER_PRESSURE_SENSOR_2 (11)

/*
 * @brief Pin of the front left release solenoid.
 *
 * PWM duty cycle controlled by timer 4 (TCCR4B).
 *
 */
#define PIN_RELEASE_SOLENOID_FRONT_LEFT (6)

/*
 * @brief Pin of the front right release solenoid.
 *
 * PWM duty cycle controlled by timer 4 (TCCR4B).
 *
 */
#define PIN_RELEASE_SOLENOID_FRONT_RIGHT (8)

/*
 * @brief Pin of the front left pressure sensor.
 *
 */
#define PIN_PRESSURE_SENSOR_FRONT_LEFT (14)

/*
 * @brief Pin of the front right pressure sensor.
 *
 */
#define PIN_PRESSURE_SENSOR_FRONT_RIGHT (13)

/*
 * @brief Pin of the wheel pressure check 1 (PCK1) signal.
 *
 */
#define PIN_WHEEL_PRESSURE_CHECK_1 (13)

/*
 * @brief Pin of the wheel pressure check 2 (PCK2) signal.
 *
 */
#define PIN_WHEEL_PRESSURE_CHECK_2 (12)

/*
 * @brief Pin of the brake light.
 *
 */
#define PIN_BRAKE_LIGHT (48)

/*
 * @brief PWM value to turn a solenoid off.
 *
 */
#define SOLENOID_PWM_OFF (0)

/*
 * @brief PWM value to turn a solenoid on.
 *
 */
#define SOLENOID_PWM_ON (255)

#ifdef GLOBAL_DEFINED
MCP_CAN g_control_can(PIN_CAN_CHIP_SELECT);

  #define EXTERN
#else
extern MCP_CAN g_control_can;

  #define EXTERN extern
#endif

EXTERN volatile brake_control_state_s g_brake_control_state;

EXTERN pid_s g_pid;

#endif /* _OSCC_BRAKE_GLOBALS_H_ */
