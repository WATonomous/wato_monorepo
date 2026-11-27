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

// Firmware settings: timing, PPS lock, reporting. Pins are in cs_pins.h.

#ifndef CS_CONFIG_H_
#define CS_CONFIG_H_

#include "cs/cs_net.h"

#define CS_FW_VERSION "1.0.0"

// ---------------------------------------------------------------------------------------------
// Trigger timing
#define CS_PULSES_PER_SECOND 20  // 50 ms: camera 20 fps, lidar 1200 rpm (issue #561)
#define CS_TRIG_PULSE_US 1000  // trigger pulse width
// Strobes are counted for a pulse until this long after it; its TRG line is sent when the window
// closes.
#define CS_STROBE_WINDOW_US 5000

// ---------------------------------------------------------------------------------------------
// PPS lock (see app/include/cs/cs_sched.h)
#define CS_PPS_LOCK_COUNT 2  // consecutive valid 1 s intervals before locking
#define CS_PPS_LOCK_TOL_PPM 100  // locked: an edge more than 100 us off the prediction is a glitch
#define CS_PPS_ACQUIRE_TOL_PPM 20000  // unlocked: interval within 2 % of nominal (HSI is +-1 %)
#define CS_PPS_TIMEOUT_MS 1500  // locked -> holdover after 1.5 s without a valid edge
#define CS_MIN_LEAD_US 20
#define CS_RESCHEDULE_GUARD_US 500

// ---------------------------------------------------------------------------------------------
// Reporting. Both transports carry the same lines; either can be turned off.
#define CS_UART_ENABLE 1  // ST-LINK virtual COM port (USART3), shows up as /dev/ttyACM*
#define CS_UART_BAUD 115200

#define CS_NET_ENABLE 1  // UDP over the NUCLEO's Ethernet port
// Static address on the sensor network (10.8.0.0/16, br0 on the car), outside the dnsmasq DHCP
// range 10.8.0.100-200 and not used by another device in src/interfacing/INTERFACING_README.md.
// TODO(camera_trigger open item A2): confirm and reserve it there.
#define CS_NET_IP CS_IP4(10, 8, 0, 60)
#define CS_NET_DEST_IP CS_IP4(10, 8, 0, 1)  // the car computer
#define CS_NET_DEST_PORT 5800  // camera_trigger udp_port
#define CS_NET_SRC_PORT 5800
#define CS_NET_ARP_RETRY_MS 1000  // ARP request period while the computer's MAC is unknown
#define CS_NET_ARP_REFRESH_MS 60000  // re-resolve periodically in case the computer changes

#define CS_STAT_ENABLE 1  // one STAT line per second, after the TRG line of pulse_idx 0

// ---------------------------------------------------------------------------------------------
// Checks
_Static_assert(CS_PULSES_PER_SECOND >= 1 && CS_PULSES_PER_SECOND <= 1000, "CS_PULSES_PER_SECOND out of range");
_Static_assert(
  CS_TRIG_PULSE_US > 0 && CS_TRIG_PULSE_US * CS_PULSES_PER_SECOND < 1000000 / 2,
  "CS_TRIG_PULSE_US must be shorter than half the pulse period");
_Static_assert(
  CS_STROBE_WINDOW_US > CS_MIN_LEAD_US && CS_STROBE_WINDOW_US * CS_PULSES_PER_SECOND < 1000000 / 2,
  "CS_STROBE_WINDOW_US must be shorter than half the pulse period");
_Static_assert(
  CS_PPS_LOCK_TOL_PPM < CS_RESCHEDULE_GUARD_US,
  "the lock tolerance must be below the reschedule guard, or an early PPS edge skips slot 0");
_Static_assert(CS_UART_ENABLE || CS_NET_ENABLE, "enable at least one report transport");

#endif  // CS_CONFIG_H_
