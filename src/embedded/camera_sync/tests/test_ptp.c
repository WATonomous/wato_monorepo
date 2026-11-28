// Copyright (c) 2025-present WATonomous. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "cs/cs_ptp.h"
#include "cs_test.h"

static void test_sync_and_follow_up(void)
{
  uint8_t msg[44] = {0};
  msg[1] = 2;
  msg[3] = sizeof(msg);
  msg[4] = 3;  // domain
  msg[6] = 2;  // two-step flag
  msg[15] = 7;  // correction in 2^-16 ns units
  msg[20] = 0x12;  // clock identity
  msg[29] = 5;  // source port
  msg[30] = 0x12;
  msg[31] = 0x34;
  msg[39] = 42;  // seconds
  msg[43] = 9;  // nanoseconds
  cs_ptp_message_t result;
  CHECK(cs_ptp_decode(msg, sizeof(msg), &result));
  CHECK_EQ_INT(result.type, CS_PTP_SYNC);
  CHECK_EQ_INT(result.domain, 3);
  CHECK(result.two_step);
  CHECK_EQ_INT(result.correction_scaled_ns, 7);
  CHECK_EQ_INT(result.source_clock[0], 0x12);
  CHECK_EQ_INT(result.source_port, 5);
  CHECK_EQ_INT(result.sequence, 0x1234);
  CHECK_EQ_INT(result.seconds, 42);
  CHECK_EQ_INT(result.nanoseconds, 9);

  memset(msg + 8, 0xff, 8);  // Signed correction field, in scaled nanoseconds.
  CHECK(cs_ptp_decode(msg, sizeof(msg), &result));
  CHECK_EQ_INT(result.correction_scaled_ns, -1);
  msg[0] = 8;
  CHECK(cs_ptp_decode(msg, sizeof(msg), &result));
  CHECK_EQ_INT(result.type, CS_PTP_FOLLOW_UP);
  CHECK(!cs_ptp_decode(msg, sizeof(msg) - 1, &result));
  msg[40] = 0x3b;
  msg[41] = 0x9a;
  msg[42] = 0xca;
  msg[43] = 0x00;  // 1,000,000,000 ns is not a valid fractional timestamp.
  CHECK(!cs_ptp_decode(msg, sizeof(msg), &result));
}

static void test_delay_response_identity_and_validation(void)
{
  uint8_t msg[54] = {0};
  msg[0] = 9;
  msg[1] = 2;
  msg[3] = sizeof(msg);
  msg[44] = 0xab;
  msg[53] = 7;
  cs_ptp_message_t result;
  CHECK(cs_ptp_decode(msg, sizeof(msg), &result));
  CHECK_EQ_INT(result.type, CS_PTP_DELAY_RESP);
  CHECK_EQ_INT(result.requesting_clock[0], 0xab);
  CHECK_EQ_INT(result.requesting_port, 7);
  msg[3] = 44;
  CHECK(!cs_ptp_decode(msg, sizeof(msg), &result));
  msg[3] = sizeof(msg);
  msg[1] = 1;
  CHECK(!cs_ptp_decode(msg, sizeof(msg), &result));
  msg[1] = 2;
  msg[0] = 0x0f;
  CHECK(!cs_ptp_decode(msg, sizeof(msg), &result));
}

static void test_announce_time_properties(void)
{
  uint8_t msg[64] = {0};
  msg[0] = 11;
  msg[1] = 2;
  msg[3] = sizeof(msg);
  msg[7] = 0x0c;  // PTP time scale and UTC offset valid.
  msg[45] = 37;
  msg[48] = 6;  // grandmaster clock class
  msg[53] = 0x34;
  cs_ptp_message_t result;
  CHECK(cs_ptp_decode(msg, sizeof(msg), &result));
  CHECK_EQ_INT(result.type, CS_PTP_ANNOUNCE);
  CHECK(result.ptp_timescale);
  CHECK(result.utc_offset_valid);
  CHECK_EQ_INT(result.utc_offset, 37);
  CHECK_EQ_INT(result.grandmaster_clock_class, 6);
  CHECK_EQ_INT(result.grandmaster_identity[0], 0x34);
  CHECK(!cs_ptp_decode(msg, sizeof(msg) - 1, &result));
}

int main(void)
{
  RUN(test_sync_and_follow_up);
  RUN(test_delay_response_identity_and_validation);
  RUN(test_announce_time_properties);
  TEST_MAIN_END();
}
