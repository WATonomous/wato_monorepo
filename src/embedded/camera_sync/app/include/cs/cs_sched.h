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

// Trigger pulse scheduler: decides when each pulse fires and keeps the pulse train locked to PPS.
//
// Portable C, no hardware access: the port calls it from the timer interrupt with raw tick values
// of a free-running 32-bit counter, and programs the compare register with the result. The same
// code runs in the host unit tests with a simulated timer.
//
// Model: a grid of slots. Second `sec` starts at its slot 0, and slot `idx` of that second fires
// at anchor_tick + ((sec - anchor_sec) * N + idx) * rate / N, where rate is the measured number
// of ticks per second. Every accepted PPS edge moves the anchor to (that second, captured tick),
// so the grid follows PPS exactly; between edges it extrapolates with the measured rate.
//
// States:
//   FREERUN   no PPS lock yet: the grid starts at an arbitrary tick, nominal rate.
//   LOCKED    the grid follows PPS: slot 0 of every second is the PPS edge.
//   HOLDOVER  PPS lost after being locked: the grid continues with the last rate, so pulses
//             stay close to where PPS would be. pps_locked = 0 in the reports.
//
// Slot 0 of a locked second fires at the predicted edge (previous edge + measured rate), because
// the edge itself is only known after it happened. The difference between prediction and edge is
// reported as the phase error (STAT line, pps_err_ns).

#ifndef CS_SCHED_H_
#define CS_SCHED_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    CS_SYNC_FREERUN = 0,
    CS_SYNC_HOLDOVER = 1,
    CS_SYNC_LOCKED = 2,
  } cs_sync_state_t;

  typedef struct
  {
    uint32_t pulses_per_second;  // N, pulses in one second (20)
    uint32_t tick_hz;  // nominal timer tick rate
    uint32_t lock_count;  // consecutive valid PPS intervals needed to lock (>= 1)
    uint32_t lock_tol_ppm;  // locked: max phase error of an edge, in ppm of a second
    uint32_t acquire_tol_ppm;  // unlocked: max PPS interval error against tick_hz
    uint32_t pps_timeout_ms;  // locked -> holdover after this long without an accepted edge
    uint32_t min_lead_us;  // a pulse is never scheduled closer than this to "now"
    uint32_t reschedule_guard_us;  // a pending pulse this close to firing is never moved
  } cs_sched_cfg_t;

  // One fired pulse, as reported in a TRG line.
  typedef struct
  {
    uint32_t tick;  // counter value the pulse fired at
    uint32_t seq;
    uint32_t pps_count;
    uint32_t pulse_idx;
    bool pps_locked;
  } cs_pulse_t;

  typedef struct
  {
    uint32_t pps_edges;  // every captured edge
    uint32_t pps_glitches;  // edges rejected while locked (off the predicted second)
    uint32_t skipped_pulses;  // slots of the grid that were not fired
    uint32_t realigns;  // times the grid jumped to a new PPS phase
    int32_t last_err_ticks;  // edge minus predicted second, for the last accepted locked edge
    bool have_err;
  } cs_sched_stats_t;

  typedef struct
  {
    cs_sched_cfg_t cfg;
    uint32_t min_lead_ticks;
    uint32_t guard_ticks;
    uint32_t timeout_ticks;

    int64_t rate_q8;  // ticks per second, times 256
    uint32_t anchor_tick;
    uint32_t anchor_sec;
    cs_sync_state_t state;

    uint32_t pend_tick;
    uint32_t pend_sec;
    uint32_t pend_idx;
    bool restart_from_anchor;  // the grid jumped: the next search starts at (anchor_sec, 0)

    bool have_fired;
    uint32_t fired_tick;
    uint32_t fired_sec;
    uint32_t seq;

    bool have_cap;
    uint32_t last_cap;
    uint32_t acq_streak;
    uint32_t last_good_tick;

    cs_sched_stats_t stats;
  } cs_sched_t;

  // Starts a free-running grid whose first pulse fires about 10 ms after `now`.
  void cs_sched_init(cs_sched_t * s, const cs_sched_cfg_t * cfg, uint32_t now);

  // The pending pulse fired at its scheduled tick. Fills `out` with its report fields and schedules
  // the next one (read it with cs_sched_pending_tick). `now` is the counter value after the edge.
  void cs_sched_fired(cs_sched_t * s, uint32_t now, cs_pulse_t * out);

  // A PPS edge was captured at `cap`. Returns true if the pending pulse moved, so the port must
  // program the compare register again.
  bool cs_sched_pps(cs_sched_t * s, uint32_t cap, uint32_t now);

  // The port could not fire the pending pulse on time (its tick had already passed when the compare
  // was programmed). Counts it as skipped and schedules the next slot.
  void cs_sched_skip_pending(cs_sched_t * s, uint32_t now);

  static inline uint32_t cs_sched_pending_tick(const cs_sched_t * s)
  {
    return s->pend_tick;
  }

  static inline cs_sync_state_t cs_sched_state(const cs_sched_t * s)
  {
    return s->state;
  }

  // Converts a tick count to nanoseconds with the measured rate.
  int32_t cs_sched_ticks_to_ns(const cs_sched_t * s, int32_t ticks);

  // Converts microseconds to ticks with the nominal rate.
  uint32_t cs_sched_us_to_ticks(const cs_sched_t * s, uint32_t us);

#ifdef __cplusplus
}
#endif

#endif  // CS_SCHED_H_
