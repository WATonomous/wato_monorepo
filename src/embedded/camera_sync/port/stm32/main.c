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

// Camera trigger firmware entry point and main loop.
//
// Interrupts (by priority):
//   0  TIM2      trigger pulses, PPS capture, strobe windows (trigger.c)
//   1  EXTI      strobe inputs
//   15 SysTick   HAL millisecond tick
// Main loop: formats report lines and sends them (UART, UDP), runs the network and the LEDs.
// Nothing in the main loop can delay a pulse.

#include <string.h>

#include "board.h"
#include "cs/cs_net.h"
#include "cs/cs_report.h"
#include "cs/cs_ring.h"
#include "cs_config.h"
#include "cs_hw.h"
#include "eth_port.h"
#include "trigger.h"
#include "uart_tx.h"

#ifndef CS_RESET_ON_FAULT
  #define CS_RESET_ON_FAULT 1
#endif

static cs_ring_t g_ring;
static cs_net_t g_net;
static uint8_t g_frame[CS_NET_MAX_FRAME];
static char g_boot_line[CS_REPORT_MAX_LINE];
static size_t g_boot_len;
static bool g_boot_sent_udp;
static bool g_uart_ok;
static bool g_eth_ok;
static bool g_link;

// ---- Faults ---------------------------------------------------------------------------------

void cs_fatal(void)
{
  __disable_irq();
  cs_trigger_force_idle();
  cs_gpio_write(CS_LED_FAULT, true);
#if CS_RESET_ON_FAULT
  for (volatile uint32_t i = 0; i < 2000000u; ++i) {
  }
  NVIC_SystemReset();
#endif
  for (;;) {
  }
}

// Called by __libc_init_array from the startup file. Without crt0/crti (-nostartfiles) these hooks
// have nothing to do: this firmware has no C++ static constructors.
void _init(void);
void _fini(void);

void _init(void)
{}

void _fini(void)
{}

// HAL and CMSIS code calls this name on errors.
void Error_Handler(void)
{
  cs_fatal();
}

static void mpu_config(void)
{
  // Background region: no access outside code, SRAM, peripherals and the system space, so a
  // stray pointer faults instead of hitting external memory regions.
  MPU_Region_InitTypeDef r = {0};
  HAL_MPU_Disable();
  r.Enable = MPU_REGION_ENABLE;
  r.Number = MPU_REGION_NUMBER0;
  r.BaseAddress = 0x0;
  r.Size = MPU_REGION_SIZE_4GB;
  r.SubRegionDisable = 0x87;
  r.TypeExtField = MPU_TEX_LEVEL0;
  r.AccessPermission = MPU_REGION_NO_ACCESS;
  r.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  r.IsShareable = MPU_ACCESS_SHAREABLE;
  r.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  r.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&r);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

// ---- Reporting ------------------------------------------------------------------------------

static void send_udp(const char * line, size_t len)
{
  const size_t n = cs_net_build_udp(&g_net, (const uint8_t *)line, len, g_frame, sizeof(g_frame));
  if (n > 0u && !cs_eth_send(g_frame, n)) {
    cs_trigger_count_tx_drop();
  }
}

static void emit(const char * line, size_t len)
{
  if (g_uart_ok && !cs_uart_write(line, len)) {
    cs_trigger_count_tx_drop();
  }
  // UDP only once the computer's MAC is known; lines before that exist only on the UART.
  if (g_eth_ok && g_link && cs_net_ready(&g_net)) {
    send_udp(line, len);
  }
}

static void drain_records(void)
{
  cs_record_t rec;
  char line[CS_REPORT_MAX_LINE];
  for (int n = 0; n < 8 && cs_ring_pop(&g_ring, &rec); ++n) {
    const size_t len = cs_report_record(line, sizeof(line), &rec);
    if (len > 0u) {
      emit(line, len);
    }
  }
}

// ---- Network housekeeping (every 100 ms) ----------------------------------------------------

static void network_tick(uint32_t now_ms)
{
  static uint32_t last_arp_ms;
  static bool arp_once;
  if (!g_eth_ok) {
    return;
  }
  const bool link = cs_eth_poll_link();
  if (link != g_link) {
    g_link = link;
    cs_net_forget(&g_net);
    arp_once = false;
  }
  if (!g_link) {
    return;
  }
  const uint32_t period = cs_net_ready(&g_net) ? CS_NET_ARP_REFRESH_MS : CS_NET_ARP_RETRY_MS;
  if (!arp_once || now_ms - last_arp_ms >= period) {
    const size_t n = cs_net_build_arp_request(&g_net, g_frame, sizeof(g_frame));
    cs_eth_send(g_frame, n);
    last_arp_ms = now_ms;
    arp_once = true;
  }
  // The BOOT line goes out over UDP once, as soon as the computer can be reached.
  if (!g_boot_sent_udp && cs_net_ready(&g_net)) {
    send_udp(g_boot_line, g_boot_len);
    g_boot_sent_udp = true;
  }
}

// ---- LEDs (every 100 ms) --------------------------------------------------------------------

static void leds_tick(uint32_t now_ms)
{
  static uint32_t last_problems;
  static uint32_t fault_until_ms;
  cs_trigger_status_t st;
  cs_trigger_status(&st);

  // LOCK: on = locked to PPS, 1 Hz blink = holdover, short flash every 2 s = free-running.
  bool lock_on = false;
  switch (st.state) {
    case CS_SYNC_LOCKED:
      lock_on = true;
      break;
    case CS_SYNC_HOLDOVER:
      lock_on = (now_ms % 1000u) < 500u;
      break;
    default:
      lock_on = (now_ms % 2000u) < 100u;
      break;
  }
  cs_gpio_write(CS_LED_LOCK, lock_on);

  // LINK: on = computer reachable, 2 Hz blink = Ethernet link but no ARP reply yet.
  const bool ready = g_link && cs_net_ready(&g_net);
  cs_gpio_write(CS_LED_LINK, ready || (g_link && (now_ms % 500u) < 250u));

  // FAULT: on for 1 s after a dropped report line, a skipped pulse or a PPS glitch.
  if (st.problems != last_problems) {
    last_problems = st.problems;
    fault_until_ms = now_ms + 1000u;
  }
  cs_gpio_write(CS_LED_FAULT, (int32_t)(fault_until_ms - now_ms) > 0);
}

// ---- Main -----------------------------------------------------------------------------------

static void leds_init(void)
{
  cs_gpio_init(CS_LED_LOCK, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
  cs_gpio_init(CS_LED_LINK, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
  cs_gpio_init(CS_LED_FAULT, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
}

static void read_uid(uint32_t uid[3])
{
  const volatile uint32_t * p = (const volatile uint32_t *)UID_BASE;
  uid[0] = p[0];
  uid[1] = p[1];
  uid[2] = p[2];
}

int main(void)
{
  mpu_config();
  SCB_EnableICache();  // the data cache stays off: Ethernet DMA buffers need no maintenance
  HAL_Init();
  board_early_init();
  const cs_clock_source_t clock = board_clock_init();
  leds_init();

  // BOOT,1,<version>+<board>.<clock>, e.g. "1.0.0+h753zi.hse"
  char version[48];
  strcpy(version, CS_FW_VERSION "+" CS_BOARD_NAME);
  strcat(version, clock == CS_CLOCK_HSE ? ".hse" : ".hsi");
  g_boot_len = cs_report_boot(g_boot_line, sizeof(g_boot_line), version);

  cs_ring_init(&g_ring);
  g_uart_ok = CS_UART_ENABLE && cs_uart_init();
  if (g_uart_ok) {
    cs_uart_write(g_boot_line, g_boot_len);
  }

  if (CS_NET_ENABLE) {
    uint32_t uid[3];
    read_uid(uid);
    cs_net_cfg_t cfg = {
      .mac = {0},
      .ip = CS_NET_IP,
      .dest_ip = CS_NET_DEST_IP,
      .dest_port = CS_NET_DEST_PORT,
      .src_port = CS_NET_SRC_PORT};
    cs_net_mac_from_uid(uid, cfg.mac);
    cs_net_init(&g_net, &cfg);
    g_eth_ok = cs_eth_init(cfg.mac);
  }

  cs_trigger_start(&g_ring, board_tim2_clock_hz());

  uint32_t last_tick_ms = HAL_GetTick();
  for (;;) {
    drain_records();
    if (g_uart_ok) {
      cs_uart_pump();
    }
    if (g_eth_ok) {
      cs_eth_poll_rx(&g_net);
    }
    const uint32_t now_ms = HAL_GetTick();
    if (now_ms - last_tick_ms >= 100u) {
      last_tick_ms = now_ms;
      network_tick(now_ms);
      leds_tick(now_ms);
    }
  }
}
