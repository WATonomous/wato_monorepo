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

// Report lines must match the camera_trigger node's protocol (PR #611,
// src/perception/camera_trigger/docs/serial_protocol.md) byte for byte. The expected strings are
// the examples from that document, with their checksums.

#include "cs/cs_report.h"
#include "cs/cs_ring.h"
#include "cs_test.h"

static void test_boot_matches_protocol_example(void)
{
  char buf[CS_REPORT_MAX_LINE];
  const size_t n = cs_report_boot(buf, sizeof(buf), "v1.0.0");
  CHECK_EQ_STR(buf, "BOOT,1,v1.0.0*60\r\n");
  CHECK_EQ_INT(n, strlen("BOOT,1,v1.0.0*60\r\n"));
}

static void test_trg_matches_protocol_examples(void)
{
  char buf[CS_REPORT_MAX_LINE];
  cs_pulse_t p = {.tick = 0, .seq = 1234, .pps_count = 61, .pulse_idx = 14, .pps_locked = true};
  cs_report_trg(buf, sizeof(buf), &p, true, 0xFFF);
  CHECK_EQ_STR(buf, "TRG,1,1234,61,14,1,FFF*01\r\n");

  p.seq = 1235;
  p.pulse_idx = 15;
  cs_report_trg(buf, sizeof(buf), &p, true, 0xFFB);
  CHECK_EQ_STR(buf, "TRG,1,1235,61,15,1,FFB*05\r\n");

  p.seq = 1236;
  p.pulse_idx = 16;
  p.pps_locked = false;
  cs_report_trg(buf, sizeof(buf), &p, false, 0);
  CHECK_EQ_STR(buf, "TRG,1,1236,61,16,0,---*6B\r\n");
}

static void test_trg_mask_is_three_hex_digits(void)
{
  char buf[CS_REPORT_MAX_LINE];
  cs_pulse_t p = {.tick = 0, .seq = 0, .pps_count = 0, .pulse_idx = 0, .pps_locked = false};
  cs_report_trg(buf, sizeof(buf), &p, true, 0x005);
  CHECK(strncmp(buf, "TRG,1,0,0,0,0,005*", 18) == 0);
  cs_report_trg(buf, sizeof(buf), &p, true, 0xF005);  // bits above STB12 are dropped
  CHECK(strncmp(buf, "TRG,1,0,0,0,0,005*", 18) == 0);
}

static void test_max_values_fit(void)
{
  char buf[CS_REPORT_MAX_LINE];
  cs_pulse_t p = {.tick = 0, .seq = 4294967295u, .pps_count = 4294967295u, .pulse_idx = 999, .pps_locked = true};
  CHECK(cs_report_trg(buf, sizeof(buf), &p, true, 0xFFF) > 0);
  CHECK(strncmp(buf, "TRG,1,4294967295,4294967295,999,1,FFF*", 38) == 0);
}

static void test_stat_line(void)
{
  char buf[CS_REPORT_MAX_LINE];
  cs_stat_t st = {
    .uptime_s = 3600,
    .state = CS_SYNC_LOCKED,
    .pps_err_ns = -125,
    .pps_glitches = 2,
    .skipped_pulses = 0,
    .stray_strobes = 7,
    .tx_drops = 1};
  cs_report_stat(buf, sizeof(buf), &st);
  // Checksum computed independently of the formatter.
  const char * body = "STAT,1,3600,LOCKED,-125,2,0,7,1";
  unsigned sum = 0;
  for (const char * c = body; *c; ++c) {
    sum ^= (unsigned char)*c;
  }
  char expected[64];
  snprintf(expected, sizeof(expected), "%s*%02X\r\n", body, sum);
  CHECK_EQ_STR(buf, expected);

  st.state = CS_SYNC_HOLDOVER;
  st.pps_err_ns = -2147483647 - 1;
  CHECK(cs_report_stat(buf, sizeof(buf), &st) > 0);
  CHECK(strstr(buf, ",HOLDOVER,-2147483648,") != NULL);
}

static void test_boot_sanitizes_version(void)
{
  char buf[CS_REPORT_MAX_LINE];
  cs_report_boot(buf, sizeof(buf), "1.0,0*x\n");
  CHECK(strncmp(buf, "BOOT,1,1.00x*", 13) == 0);
  cs_report_boot(buf, sizeof(buf), "");
  CHECK(strncmp(buf, "BOOT,1,?*", 9) == 0);
}

static void test_small_buffer_returns_zero(void)
{
  char buf[12];
  cs_pulse_t p = {.tick = 0, .seq = 1, .pps_count = 1, .pulse_idx = 1, .pps_locked = true};
  CHECK_EQ_INT(cs_report_trg(buf, sizeof(buf), &p, true, 0), 0);
  CHECK_EQ_INT(cs_report_trg(NULL, 0, &p, true, 0), 0);
}

static void test_ring_fifo_and_full(void)
{
  static cs_ring_t ring;
  cs_ring_init(&ring);
  cs_record_t rec;
  memset(&rec, 0, sizeof(rec));
  for (uint32_t i = 0; i < CS_RING_SIZE; ++i) {
    rec.u.trg.pulse.seq = i;
    CHECK(cs_ring_push(&ring, &rec));
  }
  CHECK(!cs_ring_push(&ring, &rec));
  cs_record_t out;
  for (uint32_t i = 0; i < CS_RING_SIZE; ++i) {
    CHECK(cs_ring_pop(&ring, &out));
    CHECK_EQ_INT(out.u.trg.pulse.seq, i);
  }
  CHECK(!cs_ring_pop(&ring, &out));
}

int main(void)
{
  RUN(test_boot_matches_protocol_example);
  RUN(test_trg_matches_protocol_examples);
  RUN(test_trg_mask_is_three_hex_digits);
  RUN(test_max_values_fit);
  RUN(test_stat_line);
  RUN(test_boot_sanitizes_version);
  RUN(test_small_buffer_returns_zero);
  RUN(test_ring_fifo_and_full);
  TEST_MAIN_END();
}
