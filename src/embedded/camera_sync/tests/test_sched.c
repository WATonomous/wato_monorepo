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

// Scheduler tests against a simulated timer: a 32-bit counter running at a slightly wrong rate
// (crystal error), PPS edges at whole seconds of true time with jitter, interrupt latency, and
// faults (PPS loss, phase jump, glitch edges). Every fired pulse is checked against the ideal
// time it should have fired at.

#include <math.h>
#include <stdint.h>

#include "cs/cs_sched.h"
#include "cs_test.h"

#define N 20

typedef struct
{
  // Simulated hardware
  double tick_hz_true;  // the counter's real rate
  uint32_t offset;  // counter value at true time 0 (tests wrap-around)
  int64_t latency;  // ticks from an event to the interrupt handler reading the counter
  int64_t jitter;  // PPS edge jitter, +- ticks
  double pps_phase;  // PPS edges at true time k + pps_phase
  double pps_on_from;  // PPS present in [pps_on_from, pps_off_from) and after pps_back_at
  double pps_off_from;
  double pps_back_at;
  double phase_jump_at;  // from this time on, PPS phase += phase_jump
  double phase_jump;
  double glitch_at;  // one extra edge at this time (< 0: none)
  uint64_t rng;
} sim_cfg_t;

typedef struct
{
  double t;  // true time the pulse fired
  cs_pulse_t p;
} fired_t;

#define MAX_FIRED 4000
static fired_t g_fired[MAX_FIRED];
static int g_nfired;

static uint32_t rng_next(uint64_t * s)
{
  *s ^= *s << 13;
  *s ^= *s >> 7;
  *s ^= *s << 17;
  return (uint32_t)(*s >> 16);
}

static uint32_t counter(const sim_cfg_t * c, int64_t abs_tick)
{
  return (uint32_t)((uint64_t)abs_tick + c->offset);
}

static bool pps_present(const sim_cfg_t * c, double t)
{
  if (t < c->pps_on_from) {
    return false;
  }
  if (t >= c->pps_off_from && t < c->pps_back_at) {
    return false;
  }
  return true;
}

static double pps_phase_at(const sim_cfg_t * c, double t)
{
  return c->pps_phase + (t >= c->phase_jump_at ? c->phase_jump : 0.0);
}

static cs_sched_cfg_t default_cfg(uint32_t tick_hz)
{
  cs_sched_cfg_t cfg = {
    .pulses_per_second = N,
    .tick_hz = tick_hz,
    .lock_count = 2,
    .lock_tol_ppm = 100,
    .acquire_tol_ppm = 20000,
    .pps_timeout_ms = 1500,
    .min_lead_us = 20,
    .reschedule_guard_us = 500};
  return cfg;
}

// Runs the simulation for `duration` seconds of true time. Returns the scheduler state at the end.
static void run(cs_sched_t * s, const cs_sched_cfg_t * cfg, sim_cfg_t * c, double duration)
{
  g_nfired = 0;
  int64_t now_abs = 0;
  cs_sched_init(s, cfg, counter(c, now_abs));

  // PPS edges: whole seconds k (plus phase). Each edge's jitter is drawn once.
  int64_t k = 0;
  bool have_next_pps = false;
  int64_t pps_abs = INT64_MAX;
  bool glitch_done = c->glitch_at < 0;
  const int64_t end_abs = (int64_t)(duration * c->tick_hz_true);

  while (now_abs < end_abs) {
    // Next pulse, from the pending compare value.
    const int64_t pulse_abs = now_abs + (int32_t)(cs_sched_pending_tick(s) - counter(c, now_abs));

    // Next PPS edge that is present.
    while (!have_next_pps) {
      const double t = (double)k + pps_phase_at(c, (double)k);
      if (t > duration + 2.0) {
        pps_abs = INT64_MAX;
        have_next_pps = true;
        break;
      }
      const int64_t a = llround(t * c->tick_hz_true) +
                        (c->jitter > 0 ? (int64_t)(rng_next(&c->rng) % (2 * c->jitter + 1)) - c->jitter : 0);
      ++k;
      if (a >= now_abs && pps_present(c, t)) {
        pps_abs = a;
        have_next_pps = true;
      }
    }
    const int64_t glitch_abs = glitch_done ? INT64_MAX : llround(c->glitch_at * c->tick_hz_true);

    if (pulse_abs <= pps_abs && pulse_abs <= glitch_abs) {
      CHECK(pulse_abs >= now_abs);
      now_abs = pulse_abs;
      cs_pulse_t p;
      cs_sched_fired(s, counter(c, now_abs + c->latency), &p);
      CHECK_EQ_INT(p.tick, counter(c, now_abs));
      if (g_nfired < MAX_FIRED) {
        g_fired[g_nfired].t = (double)now_abs / c->tick_hz_true;
        g_fired[g_nfired].p = p;
        g_nfired++;
      }
    } else if (glitch_abs <= pps_abs) {
      now_abs = glitch_abs;
      glitch_done = true;
      cs_sched_pps(s, counter(c, glitch_abs), counter(c, glitch_abs + c->latency));
    } else {
      now_abs = pps_abs;
      cs_sched_pps(s, counter(c, pps_abs), counter(c, pps_abs + c->latency));
      have_next_pps = false;
    }
  }
}

static sim_cfg_t default_sim(double tick_hz_true)
{
  sim_cfg_t c = {
    .tick_hz_true = tick_hz_true,
    .offset = 0xF0000000u,  // wraps after ~1.3 s at 200 MHz
    .latency = 200,
    .jitter = 4,  // +-20 ns at 200 MHz
    .pps_phase = 0.0,
    .pps_on_from = 0.5,
    .pps_off_from = 1e9,
    .pps_back_at = 1e9,
    .phase_jump_at = 1e9,
    .phase_jump = 0.0,
    .glitch_at = -1.0,
    .rng = 0x9E3779B97F4A7C15ull};
  return c;
}

typedef struct
{
  int locked_pulses;
  int unlocked_pulses;
  double max_err_locked;  // seconds, against the ideal PPS grid
  double first_lock_t;
  double max_gap;  // largest time between consecutive pulses
  double min_gap;
  int seq_errors;
  int idx_errors;  // consecutive locked pulses not following idx+1 / wrap with pps_count+1
  int offset_changes;  // times pps_count - gps_second changed between locked pulses
} summary_t;

// Checks pulses fired in [t_from, t_to). For locked pulses, the GPS second is recovered as the
// node does: pps_count + K, with K learned from the first locked pulse (and re-learned when it
// changes, which is counted).
static summary_t summarize(const sim_cfg_t * c, double t_from, double t_to)
{
  summary_t r = {0};
  r.first_lock_t = -1;
  r.min_gap = 1e9;
  bool have_k = false;
  int64_t k_off = 0;
  const fired_t * prev = NULL;
  for (int i = 0; i < g_nfired; ++i) {
    const fired_t * f = &g_fired[i];
    if (i > 0) {
      const fired_t * b = &g_fired[i - 1];
      if (f->p.seq != b->p.seq + 1u) {
        r.seq_errors++;
      }
      const double gap = f->t - b->t;
      if (f->t >= t_from && f->t < t_to) {
        r.max_gap = fmax(r.max_gap, gap);
        r.min_gap = fmin(r.min_gap, gap);
      }
    }
    if (f->t < t_from || f->t >= t_to) {
      prev = NULL;
      continue;
    }
    if (!f->p.pps_locked) {
      r.unlocked_pulses++;
      prev = NULL;
      continue;
    }
    if (r.first_lock_t < 0) {
      r.first_lock_t = f->t;
    }
    r.locked_pulses++;
    const double phase = pps_phase_at(c, f->t);
    const double sec_start = f->t - phase - (double)f->p.pulse_idx / N;
    const int64_t gps_sec = llround(sec_start);
    const int64_t k = gps_sec - (int64_t)f->p.pps_count;
    if (!have_k || k != k_off) {
      if (have_k) {
        r.offset_changes++;
      }
      have_k = true;
      k_off = k;
    }
    const double ideal = (double)gps_sec + phase + (double)f->p.pulse_idx / N;
    r.max_err_locked = fmax(r.max_err_locked, fabs(f->t - ideal));
    if (prev != NULL) {
      const bool next_in_sec = f->p.pps_count == prev->p.pps_count && f->p.pulse_idx == prev->p.pulse_idx + 1u;
      const bool next_sec =
        f->p.pps_count == prev->p.pps_count + 1u && f->p.pulse_idx == 0u && prev->p.pulse_idx == N - 1u;
      if (!next_in_sec && !next_sec) {
        r.idx_errors++;
      }
    }
    prev = f;
  }
  return r;
}

static void test_locks_and_stays_on_grid(void)
{
  const cs_sched_cfg_t cfg = default_cfg(200000000u);
  sim_cfg_t c = default_sim(200000000.0 * (1.0 + 37e-6));
  cs_sched_t s;
  run(&s, &cfg, &c, 60.0);

  CHECK_EQ_INT(cs_sched_state(&s), CS_SYNC_LOCKED);
  const summary_t r = summarize(&c, 0.0, 60.0);
  CHECK_EQ_INT(r.seq_errors, 0);
  // PPS from 0.5 s: edges at 1, 2, 3 -> two valid intervals at t = 3 s.
  CHECK(r.first_lock_t > 3.0 && r.first_lock_t < 3.2);
  CHECK(r.min_gap > 0.5 / N);
  CHECK(r.max_gap < 2.0 / N);  // one longer gap where the free-running train jumps to the PPS phase

  const summary_t locked = summarize(&c, 4.0, 60.0);
  CHECK_EQ_INT(locked.unlocked_pulses, 0);
  CHECK_EQ_INT(locked.locked_pulses, 56 * N);
  CHECK_EQ_INT(locked.idx_errors, 0);
  CHECK_EQ_INT(locked.offset_changes, 0);
  CHECK(locked.max_err_locked < 0.2e-6);
  CHECK(locked.min_gap > 0.999 / N && locked.max_gap < 1.001 / N);
  CHECK_EQ_INT(s.stats.skipped_pulses, 0);
  CHECK_EQ_INT(s.stats.pps_glitches, 0);
  CHECK_EQ_INT(s.stats.realigns, 1);
  CHECK(s.stats.have_err);
  CHECK(abs(cs_sched_ticks_to_ns(&s, s.stats.last_err_ticks)) < 200);
}

static void test_f767_tick_rate_and_hsi_error(void)
{
  // 108 MHz timer clock running 0.8 % fast (HSI fallback): still locks, still exact.
  const cs_sched_cfg_t cfg = default_cfg(108000000u);
  sim_cfg_t c = default_sim(108000000.0 * 1.008);
  cs_sched_t s;
  run(&s, &cfg, &c, 30.0);
  CHECK_EQ_INT(cs_sched_state(&s), CS_SYNC_LOCKED);
  const summary_t r = summarize(&c, 4.0, 30.0);
  CHECK_EQ_INT(r.unlocked_pulses, 0);
  CHECK_EQ_INT(r.idx_errors, 0);
  CHECK(r.max_err_locked < 0.3e-6);
  CHECK_EQ_INT(s.stats.skipped_pulses, 0);
}

static void test_free_runs_without_pps(void)
{
  const cs_sched_cfg_t cfg = default_cfg(200000000u);
  sim_cfg_t c = default_sim(200000000.0 * (1.0 - 20e-6));
  c.pps_on_from = 1e9;
  cs_sched_t s;
  run(&s, &cfg, &c, 10.0);
  CHECK_EQ_INT(cs_sched_state(&s), CS_SYNC_FREERUN);
  const summary_t r = summarize(&c, 0.0, 10.0);
  CHECK_EQ_INT(r.locked_pulses, 0);
  CHECK_EQ_INT(r.seq_errors, 0);
  CHECK(r.unlocked_pulses >= 10 * N - 1 && r.unlocked_pulses <= 10 * N);
  // pps_count advances every N pulses, pulse_idx cycles.
  for (int i = 1; i < g_nfired; ++i) {
    const cs_pulse_t * a = &g_fired[i - 1].p;
    const cs_pulse_t * b = &g_fired[i].p;
    const bool ok = (b->pps_count == a->pps_count && b->pulse_idx == a->pulse_idx + 1u) ||
                    (b->pps_count == a->pps_count + 1u && b->pulse_idx == 0u && a->pulse_idx == N - 1u);
    CHECK(ok);
  }
}

static void test_pps_loss_holdover_and_rejoin(void)
{
  const cs_sched_cfg_t cfg = default_cfg(200000000u);
  sim_cfg_t c = default_sim(200000000.0 * (1.0 + 37e-6));
  c.pps_off_from = 20.5;  // last edge at 20 s
  c.pps_back_at = 30.5;  // edges again from 31 s
  cs_sched_t s;
  run(&s, &cfg, &c, 45.0);

  const summary_t all = summarize(&c, 0.0, 45.0);
  CHECK_EQ_INT(all.seq_errors, 0);
  const summary_t steady = summarize(&c, 4.0, 45.0);
  CHECK(steady.max_gap < 1.001 / N && steady.min_gap > 0.999 / N);  // losing PPS never moves a pulse

  // Locked until the timeout (1.5 s after the 20 s edge), then holdover.
  const summary_t before = summarize(&c, 4.0, 21.45);
  CHECK_EQ_INT(before.unlocked_pulses, 0);
  const summary_t hold = summarize(&c, 21.55, 33.0);
  CHECK_EQ_INT(hold.locked_pulses, 0);

  // Back: locks again at the third edge (33 s) without changing the second numbering.
  const summary_t after = summarize(&c, 33.1, 45.0);
  CHECK_EQ_INT(after.unlocked_pulses, 0);
  CHECK(after.max_err_locked < 0.2e-6);
  const summary_t whole = summarize(&c, 4.0, 45.0);
  CHECK_EQ_INT(whole.offset_changes, 0);
  CHECK_EQ_INT(s.stats.realigns, 1);
  CHECK_EQ_INT(s.stats.skipped_pulses, 0);

  // During holdover the grid stays within a few microseconds of where PPS would be.
  double max_hold_err = 0;
  for (int i = 0; i < g_nfired; ++i) {
    const fired_t * f = &g_fired[i];
    if (f->t > 21.0 && f->t < 33.0) {
      const double ideal = round(f->t * N) / N;
      max_hold_err = fmax(max_hold_err, fabs(f->t - ideal));
    }
  }
  CHECK(max_hold_err < 5e-6);
}

static void test_phase_jump_realigns(void)
{
  const cs_sched_cfg_t cfg = default_cfg(200000000u);
  sim_cfg_t c = default_sim(200000000.0 * (1.0 + 37e-6));
  c.phase_jump_at = 20.5;
  c.phase_jump = 0.3137;  // receiver restarted with another phase
  cs_sched_t s;
  run(&s, &cfg, &c, 40.0);

  CHECK_EQ_INT(cs_sched_state(&s), CS_SYNC_LOCKED);
  const summary_t all = summarize(&c, 0.0, 40.0);
  CHECK_EQ_INT(all.seq_errors, 0);
  CHECK(all.min_gap > 0.5 / N);
  CHECK(all.max_gap < 2.0 / N);
  CHECK(s.stats.pps_glitches >= 1);  // the off-grid edges while still locked
  CHECK_EQ_INT(s.stats.realigns, 2);

  // pps_count never goes backwards.
  for (int i = 1; i < g_nfired; ++i) {
    CHECK(g_fired[i].p.pps_count >= g_fired[i - 1].p.pps_count);
  }
  // Locked on the new phase.
  const summary_t after = summarize(&c, 26.0, 40.0);
  CHECK_EQ_INT(after.unlocked_pulses, 0);
  CHECK_EQ_INT(after.idx_errors, 0);
  CHECK(after.max_err_locked < 0.2e-6);
}

static void test_glitch_edge_is_ignored(void)
{
  const cs_sched_cfg_t cfg = default_cfg(200000000u);
  sim_cfg_t c = default_sim(200000000.0 * (1.0 + 37e-6));
  c.glitch_at = 20.437;
  cs_sched_t s;
  run(&s, &cfg, &c, 30.0);
  CHECK_EQ_INT(s.stats.pps_glitches, 1);
  const summary_t r = summarize(&c, 4.0, 30.0);
  CHECK_EQ_INT(r.unlocked_pulses, 0);
  CHECK_EQ_INT(r.idx_errors, 0);
  CHECK(r.max_err_locked < 0.2e-6);
  CHECK_EQ_INT(s.stats.realigns, 1);
}

static void test_pps_phase_not_at_counter_zero(void)
{
  // PPS edges at k + 0.73 s: pulse 0 of every second must land on the edge.
  const cs_sched_cfg_t cfg = default_cfg(200000000u);
  sim_cfg_t c = default_sim(200000000.0 * (1.0 - 12e-6));
  c.pps_phase = 0.73;
  c.offset = 0x12345678u;
  cs_sched_t s;
  run(&s, &cfg, &c, 20.0);
  const summary_t r = summarize(&c, 5.0, 20.0);
  CHECK_EQ_INT(r.unlocked_pulses, 0);
  CHECK(r.max_err_locked < 0.2e-6);
  for (int i = 0; i < g_nfired; ++i) {
    if (g_fired[i].t > 5.0 && g_fired[i].p.pulse_idx == 0u) {
      const double frac = g_fired[i].t - floor(g_fired[i].t);
      CHECK(fabs(frac - 0.73) < 1e-6);
    }
  }
}

static void test_skip_pending(void)
{
  const cs_sched_cfg_t cfg = default_cfg(200000000u);
  cs_sched_t s;
  cs_sched_init(&s, &cfg, 1000);
  const uint32_t first = cs_sched_pending_tick(&s);
  // The port found the compare already passed: the next slot is one period later.
  cs_sched_skip_pending(&s, first + 10);
  CHECK_EQ_INT(s.stats.skipped_pulses, 1);
  CHECK_EQ_INT(cs_sched_pending_tick(&s) - first, 200000000u / N);
  CHECK_EQ_INT(s.pend_idx, 1);
}

int main(void)
{
  RUN(test_locks_and_stays_on_grid);
  RUN(test_f767_tick_rate_and_hsi_error);
  RUN(test_free_runs_without_pps);
  RUN(test_pps_loss_holdover_and_rejoin);
  RUN(test_phase_jump_realigns);
  RUN(test_glitch_edge_is_ignored);
  RUN(test_pps_phase_not_at_counter_zero);
  RUN(test_skip_pending);
  TEST_MAIN_END();
}
