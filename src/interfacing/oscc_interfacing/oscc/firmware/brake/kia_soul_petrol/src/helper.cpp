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
 * @file helper.cpp
 *
 */

#include "helper.h"

#include <stdlib.h>

#include "globals.h"
#include "vehicles.h"

float interpolate(const float input, const interpolate_range_s * const range)
{
  float output = input;

  if (range != NULL) {
    output = (input - range->input_min);
    output /= (range->input_max - range->input_min);
    output *= (range->output_max - range->output_min);
    output += range->output_min;
  }

  return (output);
}

float raw_adc_to_pressure(const int input)
{
  float pressure = (float)input;
  pressure *= VOLTAGE_TO_PRESSURE_SCALAR;
  pressure += VOLTAGE_TO_PRESSURE_OFFSET;

  if (pressure < BRAKE_PRESSURE_MIN_IN_DECIBARS) {
    pressure = BRAKE_PRESSURE_MIN_IN_DECIBARS;
  } else if (pressure > BRAKE_PRESSURE_MAX_IN_DECIBARS) {
    pressure = BRAKE_PRESSURE_MAX_IN_DECIBARS;
  }

  return (pressure);
}
