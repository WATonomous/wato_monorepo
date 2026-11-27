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

// Report lines sent to the computer (protocol version 1), see docs/report_protocol.md:
//
//   BOOT,1,<fw_version>*XX
//   TRG,1,<seq>,<pps_count>,<pulse_idx>,<pps_locked>,<strobe_mask>*XX
//   STAT,1,<uptime_s>,<state>,<pps_err_ns>,<pps_glitches>,<skipped_pulses>,<stray_strobes>,<tx_drops>*XX
//
// XX is the XOR of every byte between the start of the line and '*', as two uppercase hex digits.
// Every line ends with "\r\n" and is at most CS_REPORT_MAX_LINE bytes long.

#ifndef CS_REPORT_H_
#define CS_REPORT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cs/cs_sched.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CS_REPORT_VERSION 1
#define CS_REPORT_MAX_LINE 256

  typedef struct
  {
    uint32_t uptime_s;
    cs_sync_state_t state;
    int32_t pps_err_ns;  // 0 when no locked edge has been measured yet
    uint32_t pps_glitches;
    uint32_t skipped_pulses;
    uint32_t stray_strobes;
    uint32_t tx_drops;
  } cs_stat_t;

  typedef enum
  {
    CS_RECORD_TRG = 0,
    CS_RECORD_STAT = 1,
  } cs_record_type_t;

  // What the interrupt side hands to the main loop. Formatting happens in the main loop.
  typedef struct
  {
    cs_record_type_t type;

    union
    {
      struct
      {
        cs_pulse_t pulse;
        uint16_t strobe_mask;  // bit k = strobe input STB(k+1) saw an edge for this pulse
        bool strobes_measured;  // false: the line carries "---"
      } trg;

      cs_stat_t stat;
    } u;
  } cs_record_t;

  // Each returns the line length including "\r\n" (no terminating NUL is counted, but one is
  // written), or 0 if `cap` is too small.
  size_t cs_report_boot(char * buf, size_t cap, const char * fw_version);
  size_t cs_report_trg(char * buf, size_t cap, const cs_pulse_t * p, bool strobes_measured, uint16_t strobe_mask);
  size_t cs_report_stat(char * buf, size_t cap, const cs_stat_t * st);
  size_t cs_report_record(char * buf, size_t cap, const cs_record_t * rec);

  const char * cs_report_state_name(cs_sync_state_t state);

#ifdef __cplusplus
}
#endif

#endif  // CS_REPORT_H_
