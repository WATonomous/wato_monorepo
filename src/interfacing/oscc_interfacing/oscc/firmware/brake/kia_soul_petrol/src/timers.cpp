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
 * @file timers.cpp
 *
 */

#include "timers.h"

#include <Arduino.h>

#include "brake_control.h"
#include "can_protocols/brake_can_protocol.h"
#include "communications.h"
#include "globals.h"
#include "oscc_timer.h"

/*
 * @brief Fault checking frequency. [Hz]
 *
 */
#define FAULT_CHECK_FREQUENCY_IN_HZ (1)

static void check_for_faults(void);

void start_timers(void)
{
  timer1_init(FAULT_CHECK_FREQUENCY_IN_HZ, check_for_faults);
  timer2_init(OSCC_BRAKE_REPORT_PUBLISH_FREQ_IN_HZ, publish_brake_report);
}

static void check_for_faults(void)
{
  cli();

  check_for_sensor_faults();

  sei();
}
