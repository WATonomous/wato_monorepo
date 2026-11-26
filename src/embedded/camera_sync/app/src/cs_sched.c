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

#include "cs/cs_sched.h"

#include <string.h>

// True if tick a is strictly later than tick b (valid while they are less than 2^31 ticks apart).
static bool after(uint32_t a, uint32_t b)
{
  return (int32_t)(a - b) > 0;
}

static int64_t abs64(int64_t v)
{
  return v < 0 ? -v : v;
}

static uint32_t rate_ticks(const cs_sched_t * s)
{
  return (uint32_t)(s->rate_q8 >> 8);
}

// Ticks of the measured rate that correspond to `ppm` parts per million of a second.
static uint32_t ppm_ticks(const cs_sched_t * s, uint32_t ppm)
{
  return (uint32_t)(((uint64_t)rate_ticks(s) * ppm) / 1000000u);
}

static uint32_t slot_tick(const cs_sched_t * s, uint32_t sec, uint32_t idx)
{
  const int64_t n = (int64_t)s->cfg.pulses_per_second;
  const int64_t k = (int64_t)(int32_t)(sec - s->anchor_sec) * n + (int64_t)idx;
  const int64_t num = k * s->rate_q8;
  const int64_t den = n * 256;
  const int64_t off = num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
  // Conversion of a negative value to unsigned is defined as modulo 2^32, as the counter is.
  return s->anchor_tick + (uint32_t)off;
}

static void set_pending(cs_sched_t * s, uint32_t sec, uint32_t idx, uint32_t tick)
{
  s->pend_sec = sec;
  s->pend_idx = idx;
  s->pend_tick = tick;
}

// Schedules the first slot at or after (sec, idx) that is far enough in the future and far enough
// after the last fired pulse. Slots passed over are counted as skipped when count_skips is set.
static void schedule_from(cs_sched_t * s, uint32_t sec, uint32_t idx, uint32_t now, bool count_skips)
{
  const uint32_t n = s->cfg.pulses_per_second;
  const uint32_t earliest = now + s->min_lead_ticks;
  const uint32_t half_period = rate_ticks(s) / (2u * n);

  for (uint32_t i = 0; i < 4u * n + 2u; ++i) {
    const uint32_t t = slot_tick(s, sec, idx);
    if (after(t, earliest) && (!s->have_fired || after(t, s->fired_tick + half_period))) {
      set_pending(s, sec, idx, t);
      return;
    }
    if (count_skips) {
      s->stats.skipped_pulses++;
    }
    if (++idx == n) {
      idx = 0;
      ++sec;
    }
  }

  // Only reachable if the counter jumped by seconds: start a fresh grid just ahead.
  s->anchor_sec = sec;
  s->anchor_tick = earliest + half_period;
  s->stats.realigns++;
  if (s->state == CS_SYNC_LOCKED) {
    s->state = CS_SYNC_HOLDOVER;
  }
  set_pending(s, sec, 0, s->anchor_tick);
}

static void check_timeouts(cs_sched_t * s, uint32_t now)
{
  if (s->state == CS_SYNC_LOCKED && (uint32_t)(now - s->last_good_tick) > s->timeout_ticks) {
    s->state = CS_SYNC_HOLDOVER;
  }
  if (s->have_cap && (uint32_t)(now - s->last_cap) > s->timeout_ticks) {
    s->have_cap = false;
    s->acq_streak = 0;
  }
}

static bool pending_is_imminent(const cs_sched_t * s, uint32_t now)
{
  return (int32_t)(s->pend_tick - now) < (int32_t)s->guard_ticks;
}

// After the anchor moved: recompute the pending pulse on the new grid, unless it is about to fire.
static bool reschedule(cs_sched_t * s, uint32_t now)
{
  if (pending_is_imminent(s, now)) {
    return false;  // it fires as planned; the following pulses use the new grid
  }
  const uint32_t old = s->pend_tick;
  if (s->restart_from_anchor) {
    s->restart_from_anchor = false;
    schedule_from(s, s->anchor_sec, 0, now, false);
  } else {
    schedule_from(s, s->pend_sec, s->pend_idx, now, true);
  }
  return s->pend_tick != old;
}

void cs_sched_init(cs_sched_t * s, const cs_sched_cfg_t * cfg, uint32_t now)
{
  memset(s, 0, sizeof(*s));
  s->cfg = *cfg;
  if (s->cfg.pulses_per_second == 0u) {
    s->cfg.pulses_per_second = 1u;
  }
  if (s->cfg.lock_count == 0u) {
    s->cfg.lock_count = 1u;
  }
  s->rate_q8 = (int64_t)cfg->tick_hz << 8;
  s->min_lead_ticks = cs_sched_us_to_ticks(s, cfg->min_lead_us);
  s->guard_ticks = cs_sched_us_to_ticks(s, cfg->reschedule_guard_us);
  s->timeout_ticks = (uint32_t)(((uint64_t)cfg->tick_hz * cfg->pps_timeout_ms) / 1000u);
  s->state = CS_SYNC_FREERUN;
  s->anchor_sec = 0;
  s->anchor_tick = now + cfg->tick_hz / 100u;
  set_pending(s, 0, 0, s->anchor_tick);
}

// Schedules the slot after the pending one, or the start of the new grid after a realign.
static void schedule_next(cs_sched_t * s, uint32_t now)
{
  if (s->restart_from_anchor) {
    s->restart_from_anchor = false;
    schedule_from(s, s->anchor_sec, 0, now, false);
    return;
  }
  uint32_t sec = s->pend_sec;
  uint32_t idx = s->pend_idx + 1u;
  if (idx == s->cfg.pulses_per_second) {
    idx = 0;
    ++sec;
  }
  schedule_from(s, sec, idx, now, true);
}

void cs_sched_fired(cs_sched_t * s, uint32_t now, cs_pulse_t * out)
{
  out->tick = s->pend_tick;
  out->seq = s->seq++;
  out->pps_count = s->pend_sec;
  out->pulse_idx = s->pend_idx;
  out->pps_locked = s->state == CS_SYNC_LOCKED;

  s->have_fired = true;
  s->fired_tick = s->pend_tick;
  s->fired_sec = s->pend_sec;

  check_timeouts(s, now);
  schedule_next(s, now);
}

void cs_sched_skip_pending(cs_sched_t * s, uint32_t now)
{
  s->stats.skipped_pulses++;
  check_timeouts(s, now);
  schedule_next(s, now);
}

bool cs_sched_pps(cs_sched_t * s, uint32_t cap, uint32_t now)
{
  s->stats.pps_edges++;
  const uint32_t lock_tol = ppm_ticks(s, s->cfg.lock_tol_ppm);

  // 1. Interval since the previous edge.
  bool interval_ok = false;
  uint32_t interval = 0;
  if (s->have_cap) {
    interval = cap - s->last_cap;
    const int64_t nominal_dev = (int64_t)interval - (int64_t)s->cfg.tick_hz;
    const int64_t acquire_tol = (int64_t)(((uint64_t)s->cfg.tick_hz * s->cfg.acquire_tol_ppm) / 1000000u);
    const bool near_nominal = abs64(nominal_dev) <= acquire_tol;
    const bool need_consistency = s->state == CS_SYNC_LOCKED || s->acq_streak > 0u;
    const bool consistent = !need_consistency || abs64((int64_t)interval - (int64_t)rate_ticks(s)) <= lock_tol;
    interval_ok = near_nominal && consistent;
  }
  s->have_cap = true;
  s->last_cap = cap;

  // 2. Nearest predicted second boundary.
  uint32_t best_sec = s->pend_sec;
  int32_t best_err = 0;
  int64_t best_abs = INT64_MAX;
  for (int32_t d = -1; d <= 1; ++d) {
    const uint32_t sec = s->pend_sec + (uint32_t)d;
    const int32_t err = (int32_t)(cap - slot_tick(s, sec, 0));
    if (abs64(err) < best_abs) {
      best_abs = abs64(err);
      best_err = err;
      best_sec = sec;
    }
  }

  // 3. Locked: accept edges on the predicted second, ignore the rest.
  if (s->state == CS_SYNC_LOCKED) {
    if (best_abs > (int64_t)lock_tol) {
      s->stats.pps_glitches++;
      return false;
    }
    if (interval_ok) {
      s->rate_q8 += (((int64_t)interval << 8) - s->rate_q8) / 4;
    }
    s->anchor_sec = best_sec;
    s->anchor_tick = cap;
    s->last_good_tick = cap;
    s->stats.last_err_ticks = best_err;
    s->stats.have_err = true;
    return reschedule(s, now);
  }

  // 4. Not locked: lock after lock_count consecutive valid intervals.
  if (interval_ok) {
    s->acq_streak++;
    s->rate_q8 = (int64_t)interval << 8;
  } else {
    s->acq_streak = 0;
  }
  if (s->acq_streak < s->cfg.lock_count) {
    return false;
  }

  const int64_t rejoin_tol = (int64_t)(rate_ticks(s) / (4u * s->cfg.pulses_per_second));
  if (s->state == CS_SYNC_HOLDOVER && best_abs <= rejoin_tol) {
    // The held grid is still on the PPS phase: keep the second numbering.
    s->anchor_sec = best_sec;
    s->stats.last_err_ticks = best_err;
    s->stats.have_err = true;
  } else {
    // New phase: this edge starts a new second, after every second already used.
    uint32_t newest = s->pend_sec;
    if (s->have_fired && after(s->fired_sec, newest)) {
      newest = s->fired_sec;
    }
    s->anchor_sec = newest + 1u;
    s->restart_from_anchor = true;
    s->stats.realigns++;
    s->stats.have_err = false;
  }
  s->anchor_tick = cap;
  s->state = CS_SYNC_LOCKED;
  s->last_good_tick = cap;
  return reschedule(s, now);
}

int32_t cs_sched_ticks_to_ns(const cs_sched_t * s, int32_t ticks)
{
  const uint32_t rate = rate_ticks(s);
  if (rate == 0u) {
    return 0;
  }
  return (int32_t)(((int64_t)ticks * 1000000000) / (int64_t)rate);
}

uint32_t cs_sched_us_to_ticks(const cs_sched_t * s, uint32_t us)
{
  return (uint32_t)(((uint64_t)s->cfg.tick_hz * us) / 1000000u);
}
