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

#include "trigger.h"

#include "cs/cs_pins_check.h"
#include "cs_config.h"
#include "cs_hw.h"

// ---- TIM2 channel assignment: PPS on its pin's channel, the three compares on the others ----
#define CH_PPS CS_PPS_TIM2_CHANNEL
#define CH_START (CH_PPS == 1 ? 2 : 1)
#define CH_END (CH_PPS <= 2 ? 3 : 2)
#define CH_REPORT (CH_PPS == 4 ? 3 : 4)
#define CC_FLAG(ch) (1u << (ch))  // TIM_SR_CCxIF, TIM_DIER_CCxIE and TIM_EGR_CCxG share bit ch
#define CC_OVERCAPTURE(ch) (1u << ((ch) + 8))  // TIM_SR_CCxOF
_Static_assert(CH_START != CH_PPS && CH_END != CH_PPS && CH_REPORT != CH_PPS, "TIM2 channel clash");
_Static_assert(CH_START != CH_END && CH_START != CH_REPORT && CH_END != CH_REPORT, "TIM2 channel clash");

#define IC_FILTER 0x3u  // input capture filter: 8 samples at the timer clock (constant 40-75 ns delay)

static cs_sched_t g_sched;
static cs_ring_t * g_ring;

static GPIO_TypeDef * g_trig_port;
static uint32_t g_trig_on;  // BSRR value that drives every output active
static uint32_t g_trig_off;  // BSRR value that drives every output idle
static uint32_t g_width_ticks;
static uint32_t g_window_ticks;

// Strobes: accumulated by the EXTI interrupt between a pulse and the end of its window.
static volatile uint32_t g_strobe_acc;
static volatile bool g_window_open;
static cs_pulse_t g_window_pulse;
static uint32_t g_strobe_lines;  // EXTI lines (pin numbers) used by strobe inputs
static uint8_t g_line_to_bit[16];
static volatile uint32_t g_stray_strobes;

static volatile uint32_t g_tx_drops;

static volatile uint32_t * ccr(unsigned ch)
{
  return &TIM2->CCR1 + (ch - 1u);
}

// ---- Compare programming --------------------------------------------------------------------

// Programs the START compare with the scheduler's pending pulse. If that tick has already passed
// (only possible if this code was delayed by more than the scheduling lead), the pulse is skipped
// and the next slot is tried: a late pulse would be reported with the wrong time.
static void program_start(void)
{
  for (int tries = 0; tries < 8; ++tries) {
    const uint32_t t = cs_sched_pending_tick(&g_sched);
    TIM2->SR = ~CC_FLAG(CH_START);
    *ccr(CH_START) = t;
    if ((int32_t)(t - TIM2->CNT) > 0 || (TIM2->SR & CC_FLAG(CH_START)) != 0u) {
      return;
    }
    cs_sched_skip_pending(&g_sched, TIM2->CNT);
  }
}

// Programs a one-shot compare. If the tick has already passed, raises the event in software so
// its handler still runs (late, which is harmless for pulse end and report).
static void program_oneshot(unsigned ch, uint32_t t)
{
  TIM2->SR = ~CC_FLAG(ch);
  *ccr(ch) = t;
  if ((int32_t)(t - TIM2->CNT) <= 0 && (TIM2->SR & CC_FLAG(ch)) == 0u) {
    TIM2->EGR = CC_FLAG(ch);
  }
}

// ---- Interrupt work -------------------------------------------------------------------------

static void push(const cs_record_t * rec)
{
  if (!cs_ring_push(g_ring, rec)) {
    __atomic_fetch_add(&g_tx_drops, 1u, __ATOMIC_RELAXED);
  }
}

static void push_stat(void)
{
  cs_record_t rec;
  rec.type = CS_RECORD_STAT;
  cs_stat_t * st = &rec.u.stat;
  st->uptime_s = HAL_GetTick() / 1000u;
  st->state = cs_sched_state(&g_sched);
  st->pps_err_ns = g_sched.stats.have_err ? cs_sched_ticks_to_ns(&g_sched, g_sched.stats.last_err_ticks) : 0;
  st->pps_glitches = g_sched.stats.pps_glitches;
  st->skipped_pulses = g_sched.stats.skipped_pulses;
  st->stray_strobes = g_stray_strobes;
  st->tx_drops = g_tx_drops;
  push(&rec);
}

static void close_window(void)
{
  const uint32_t mask = __atomic_exchange_n(&g_strobe_acc, 0u, __ATOMIC_RELAXED);
  g_window_open = false;
  cs_record_t rec;
  rec.type = CS_RECORD_TRG;
  rec.u.trg.pulse = g_window_pulse;
  rec.u.trg.strobe_mask = (uint16_t)mask;
  rec.u.trg.strobes_measured = CS_STROBE_USED_COUNT > 0;
  push(&rec);
  if (CS_STAT_ENABLE && g_window_pulse.pulse_idx == 0u) {
    push_stat();
  }
}

static void on_pulse(void)
{
  const uint32_t now = TIM2->CNT;
  const uint32_t fired = cs_sched_pending_tick(&g_sched);
  program_oneshot(CH_END, fired + g_width_ticks);
  if (g_window_open) {
    close_window();  // cannot happen with the configured window, kept as a guard
  }
  __atomic_store_n(&g_strobe_acc, 0u, __ATOMIC_RELAXED);
  g_window_open = true;
  cs_sched_fired(&g_sched, now, &g_window_pulse);
  program_oneshot(CH_REPORT, fired + g_window_ticks);
  program_start();
}

void TIM2_IRQHandler(void)
{
  const uint32_t sr = TIM2->SR;
  if ((sr & CC_FLAG(CH_START)) != 0u) {
    g_trig_port->BSRR = g_trig_on;  // first: every output switches in this one write
    TIM2->SR = ~CC_FLAG(CH_START);
    on_pulse();
  }
  if ((sr & CC_FLAG(CH_END)) != 0u) {
    TIM2->SR = ~CC_FLAG(CH_END);
    g_trig_port->BSRR = g_trig_off;
  }
  if ((sr & CC_FLAG(CH_REPORT)) != 0u) {
    TIM2->SR = ~CC_FLAG(CH_REPORT);
    if (g_window_open) {
      close_window();
    }
  }
  if ((sr & CC_FLAG(CH_PPS)) != 0u) {
    const uint32_t cap = *ccr(CH_PPS);
    TIM2->SR = ~(CC_FLAG(CH_PPS) | CC_OVERCAPTURE(CH_PPS));
    if (cs_sched_pps(&g_sched, cap, TIM2->CNT)) {
      program_start();
    }
  }
}

void cs_trigger_exti_irq(void)
{
  const uint32_t pending = __HAL_GPIO_EXTI_GET_IT(g_strobe_lines);
  __HAL_GPIO_EXTI_CLEAR_IT(pending);
  uint32_t bits = 0;
  uint32_t count = 0;
  for (uint32_t line = 0; line < 16u; ++line) {
    if ((pending & (1u << line)) != 0u) {
      bits |= 1u << g_line_to_bit[line];
      ++count;
    }
  }
  if (g_window_open) {
    __atomic_fetch_or(&g_strobe_acc, bits, __ATOMIC_RELAXED);
  } else {
    g_stray_strobes += count;
  }
}

// ---- Setup ----------------------------------------------------------------------------------

static IRQn_Type exti_irq(unsigned line)
{
  switch (line) {
    case 0:
      return EXTI0_IRQn;
    case 1:
      return EXTI1_IRQn;
    case 2:
      return EXTI2_IRQn;
    case 3:
      return EXTI3_IRQn;
    case 4:
      return EXTI4_IRQn;
    default:
      return line <= 9u ? EXTI9_5_IRQn : EXTI15_10_IRQn;
  }
}

static uint32_t hal_pull(int pull)
{
  return pull == CS_PULL_UP ? GPIO_PULLUP : (pull == CS_PULL_DOWN ? GPIO_PULLDOWN : GPIO_NOPULL);
}

static void setup_outputs(void)
{
  g_trig_port = cs_gpio_port(CS_PIN(CS_TRIG_PORT, 0));
  const uint32_t mask = CS_TRIG_PIN_MASK;
  g_trig_on = CS_TRIG_ACTIVE_HIGH ? mask : (mask << 16);
  g_trig_off = CS_TRIG_ACTIVE_HIGH ? (mask << 16) : mask;
  for (unsigned n = 0; n < 16u; ++n) {
    if ((mask & (1u << n)) != 0u) {
      const unsigned code = CS_PIN(CS_TRIG_PORT, n);
      cs_gpio_init(code, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);  // enables the clock
      g_trig_port->BSRR = g_trig_off;  // idle level before the pin becomes an output
      cs_gpio_init(code, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, 0);
    }
  }
}

static void setup_strobes(void)
{
  static const unsigned kStrobes[CS_STROBE_COUNT] = CS_STROBE_PIN_LIST;
  for (unsigned k = 0; k < CS_STROBE_COUNT; ++k) {
    const unsigned code = kStrobes[k];
    if (!CS_PIN_USED(code)) {
      continue;
    }
    const unsigned line = CS_PIN_NUM(code);
    cs_gpio_init(
      code,
      CS_STROBE_RISING_EDGE ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING,
      hal_pull(CS_STROBE_PULL),
      GPIO_SPEED_FREQ_LOW,
      0);
    g_line_to_bit[line] = (uint8_t)k;
    g_strobe_lines |= 1u << line;
  }
  __HAL_GPIO_EXTI_CLEAR_IT(g_strobe_lines);
  for (unsigned line = 0; line < 16u; ++line) {
    if ((g_strobe_lines & (1u << line)) != 0u) {
      HAL_NVIC_SetPriority(exti_irq(line), 1, 0);
      HAL_NVIC_EnableIRQ(exti_irq(line));
    }
  }
}

static void set_ccmr(unsigned ch, uint32_t field)
{
  volatile uint32_t * reg = ch <= 2u ? &TIM2->CCMR1 : &TIM2->CCMR2;
  const uint32_t shift = ((ch - 1u) & 1u) * 8u;
  *reg = (*reg & ~(0xFFu << shift)) | (field << shift);
}

static void setup_timer(void)
{
  cs_gpio_init(CS_PPS_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_TIM2);

  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->CR1 = 0;
  TIM2->DIER = 0;
  TIM2->PSC = 0;
  TIM2->ARR = 0xFFFFFFFFu;
  TIM2->EGR = TIM_EGR_UG;  // load PSC
  TIM2->CCER = 0;
  TIM2->CCMR1 = 0;
  TIM2->CCMR2 = 0;
  // PPS: input capture on its own input (CCxS = 01), filtered, on the configured edge.
  set_ccmr(CH_PPS, 0x01u | (IC_FILTER << 4));
  const uint32_t sh = (CH_PPS - 1u) * 4u;
  TIM2->CCER = (1u << sh) | (CS_PPS_RISING_EDGE ? 0u : (1u << (sh + 1u)));
  // START, END, REPORT stay in frozen output-compare mode with no pin: compare events only.
  TIM2->SR = 0;
  TIM2->CR1 = TIM_CR1_CEN;
}

void cs_trigger_start(cs_ring_t * ring, uint32_t tick_hz)
{
  g_ring = ring;
  setup_outputs();
  setup_strobes();
  setup_timer();

  const cs_sched_cfg_t cfg = {
    .pulses_per_second = CS_PULSES_PER_SECOND,
    .tick_hz = tick_hz,
    .lock_count = CS_PPS_LOCK_COUNT,
    .lock_tol_ppm = CS_PPS_LOCK_TOL_PPM,
    .acquire_tol_ppm = CS_PPS_ACQUIRE_TOL_PPM,
    .pps_timeout_ms = CS_PPS_TIMEOUT_MS,
    .min_lead_us = CS_MIN_LEAD_US,
    .reschedule_guard_us = CS_RESCHEDULE_GUARD_US};
  cs_sched_init(&g_sched, &cfg, TIM2->CNT);
  g_width_ticks = cs_sched_us_to_ticks(&g_sched, CS_TRIG_PULSE_US);
  g_window_ticks = cs_sched_us_to_ticks(&g_sched, CS_STROBE_WINDOW_US);

  program_start();
  TIM2->DIER = CC_FLAG(CH_START) | CC_FLAG(CH_END) | CC_FLAG(CH_REPORT) | CC_FLAG(CH_PPS);
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void cs_trigger_force_idle(void)
{
  if (g_trig_port != NULL) {
    g_trig_port->BSRR = g_trig_off;
  }
}

void cs_trigger_count_tx_drop(void)
{
  __atomic_fetch_add(&g_tx_drops, 1u, __ATOMIC_RELAXED);
}

void cs_trigger_status(cs_trigger_status_t * out)
{
  out->state = cs_sched_state(&g_sched);
  out->pps_edges = g_sched.stats.pps_edges;
  out->problems = g_tx_drops + g_sched.stats.skipped_pulses + g_sched.stats.pps_glitches;
}
