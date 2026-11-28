// Copyright (c) 2025-present WATonomous. All rights reserved.
// Licensed under the Apache License, Version 2.0.
#ifndef CS_PTP_H_
#define CS_PTP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { CS_PTP_SYNC = 0, CS_PTP_FOLLOW_UP = 8, CS_PTP_DELAY_RESP = 9, CS_PTP_ANNOUNCE = 11 };

typedef struct
{
  uint8_t type;
  uint8_t domain;
  bool two_step;
  int64_t correction_scaled_ns;  // units of 2^-16 ns
  uint8_t source_clock[8];
  uint16_t source_port;
  uint16_t sequence;
  uint64_t seconds;
  uint32_t nanoseconds;
  uint8_t requesting_clock[8];
  uint16_t requesting_port;
  bool ptp_timescale;
  bool utc_offset_valid;
  int16_t utc_offset;
  uint8_t grandmaster_clock_class;
  uint8_t grandmaster_identity[8];
} cs_ptp_message_t;

// Decode supported PTPv2 messages. The caller must validate the source, domain and sequence.
bool cs_ptp_decode(const uint8_t * data, size_t length, cs_ptp_message_t * out);

#endif
