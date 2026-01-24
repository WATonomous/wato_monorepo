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
 * @file master_cylinder.cpp
 *
 */

#include "master_cylinder.h"

#include <Arduino.h>

#include "debug.h"
#include "globals.h"
#include "helper.h"

void master_cylinder_init(void)
{
  pinMode(PIN_MASTER_CYLINDER_SOLENOID, OUTPUT);

  master_cylinder_open();
}

void master_cylinder_open(void)
{
  cli();
  analogWrite(PIN_MASTER_CYLINDER_SOLENOID, SOLENOID_PWM_OFF);
  sei();

  DEBUG_PRINTLN("Master Cylinder Open");
}

void master_cylinder_close(void)
{
  cli();
  analogWrite(PIN_MASTER_CYLINDER_SOLENOID, SOLENOID_PWM_ON);
  sei();

  DEBUG_PRINTLN("Master Cylinder Close");
}

void master_cylinder_read_pressure(master_cylinder_pressure_s * pressure)
{
  cli();
  int raw_adc_sensor_1 = analogRead(PIN_MASTER_CYLINDER_PRESSURE_SENSOR_1);
  int raw_adc_sensor_2 = analogRead(PIN_MASTER_CYLINDER_PRESSURE_SENSOR_2);
  sei();

  pressure->sensor_1_pressure = raw_adc_to_pressure(raw_adc_sensor_1);
  pressure->sensor_2_pressure = raw_adc_to_pressure(raw_adc_sensor_2);
}
