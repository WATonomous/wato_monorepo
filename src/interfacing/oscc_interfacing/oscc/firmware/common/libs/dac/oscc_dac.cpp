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
 * @file oscc_dac.cpp
 *
 */

#include "oscc_dac.h"

#include <Arduino.h>

static int32_t get_analog_sample_average(const int32_t num_samples, const uint8_t pin);

void prevent_signal_discontinuity(
  DAC_MCP49xx & dac, const int16_t num_samples, const uint8_t signal_pin_1, const uint8_t signal_pin_2)
{
  int32_t averages[2] = {0, 0};

  averages[0] = get_analog_sample_average(num_samples, signal_pin_1) << DAC_BIT_SHIFT_10BIT_TO_12BIT;

  averages[1] = get_analog_sample_average(num_samples, signal_pin_2) << DAC_BIT_SHIFT_10BIT_TO_12BIT;

  dac.outputA(averages[0]);
  dac.outputB(averages[1]);
}

static int32_t get_analog_sample_average(const int32_t num_samples, const uint8_t pin)
{
  int32_t sum = 0;
  int32_t i = 0;

  for (i = 0; i < num_samples; ++i) {
    sum += analogRead(pin);
  }

  return (sum / num_samples);
}
