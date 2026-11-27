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

// Trigger engine: TIM2 (32-bit, free-running at the timer clock) times everything.
//   PPS channel      input capture of the PPS edge (hardware latch, exact tick)
//   START channel    compare at the next pulse: set every trigger output in one BSRR write
//   END channel      compare at pulse + width: clear the outputs
//   REPORT channel   compare at pulse + strobe window: hand the TRG record to the main loop
// The TIM2 interrupt has the highest priority; nothing else ever masks it.

#ifndef TRIGGER_H_
#define TRIGGER_H_

#include <stdint.h>

#include "cs/cs_ring.h"
#include "cs/cs_sched.h"

#ifdef __cplusplus
extern "C"
{
#endif

  // Configures the pins, TIM2 and the strobe interrupts, and starts the pulse train. Records
  // (TRG, STAT) are pushed to `ring`.
  void cs_trigger_start(cs_ring_t * ring, uint32_t tick_hz);

  // Strobe EXTI interrupt work (called from the EXTI IRQ handlers).
  void cs_trigger_exti_irq(void);

  // Drives every trigger output to its idle level (fault path).
  void cs_trigger_force_idle(void);

  // Counted by the main loop for report lines it could not send; included in STAT.
  void cs_trigger_count_tx_drop(void);

  typedef struct
  {
    cs_sync_state_t state;
    uint32_t pps_edges;
    uint32_t problems;  // drops + skips + glitches: the fault LED lights when it changes
  } cs_trigger_status_t;

  void cs_trigger_status(cs_trigger_status_t * out);

#ifdef __cplusplus
}
#endif

#endif  // TRIGGER_H_
