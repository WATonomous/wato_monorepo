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
 * @file accumulator.cpp
 *
 */

#include "accumulator.h"

#include <Arduino.h>

#include "debug.h"
#include "globals.h"
#include "helper.h"
#include "vehicles.h"

void accumulator_init(void)
{
  pinMode(PIN_ACCUMULATOR_PUMP_MOTOR, OUTPUT);

  accumulator_turn_pump_off();
}

void accumulator_turn_pump_off(void)
{
  cli();
  digitalWrite(PIN_ACCUMULATOR_PUMP_MOTOR, LOW);
  sei();
}

void accumulator_turn_pump_on(void)
{
  cli();
  digitalWrite(PIN_ACCUMULATOR_PUMP_MOTOR, HIGH);
  sei();
}

float accumulator_read_pressure(void)
{
  cli();
  int raw_adc = analogRead(PIN_ACCUMULATOR_PRESSURE_SENSOR);
  sei();

  float pressure = raw_adc_to_pressure(raw_adc);

  return pressure;
}

void accumulator_maintain_pressure(void)
{
  float pressure = accumulator_read_pressure();

  if (pressure <= BRAKE_ACCUMULATOR_PRESSURE_MIN_IN_DECIBARS) {
    accumulator_turn_pump_on();
  } else if (pressure >= BRAKE_ACCUMULATOR_PRESSURE_MAX_IN_DECIBARS) {
    accumulator_turn_pump_off();
  }
}
