// Copyright (c) 2025-present WATonomous. All rights reserved.
// Licensed under the Apache License, Version 2.0.
#include "cs/cs_ptp.h"

#include <string.h>

static uint64_t read_be(const uint8_t * data, size_t length)
{
  uint64_t result = 0;
  for (size_t i = 0; i < length; ++i) {
    result = (result << 8) | data[i];
  }
  return result;
}

bool cs_ptp_decode(const uint8_t * data, size_t length, cs_ptp_message_t * out)
{
  if (data == NULL || out == NULL || length < 44u || (data[1] & 0x0fu) != 2u ||
      read_be(data + 2, 2) != length) {
    return false;
  }
  const uint8_t type = data[0] & 0x0fu;
  if (type != CS_PTP_SYNC && type != CS_PTP_FOLLOW_UP && type != CS_PTP_DELAY_RESP && type != CS_PTP_ANNOUNCE) {
    return false;
  }
  if ((type == CS_PTP_DELAY_RESP && length < 54u) || (type == CS_PTP_ANNOUNCE && length < 64u)) {
    return false;
  }
  const uint32_t nanoseconds = (uint32_t)read_be(data + 40, 4);
  if (type != CS_PTP_ANNOUNCE && nanoseconds >= 1000000000u) {
    return false;
  }
  cs_ptp_message_t decoded = {0};
  decoded.type = type;
  decoded.domain = data[4];
  decoded.two_step = (data[6] & 0x02u) != 0;
  const uint64_t correction = read_be(data + 8, 8);
  decoded.correction_scaled_ns = (correction >> 63) != 0u ? -(int64_t)(~correction) - 1 : (int64_t)correction;
  memcpy(decoded.source_clock, data + 20, sizeof(decoded.source_clock));
  decoded.source_port = (uint16_t)read_be(data + 28, 2);
  decoded.sequence = (uint16_t)read_be(data + 30, 2);
  decoded.seconds = read_be(data + 34, 6);
  decoded.nanoseconds = nanoseconds;
  if (type == CS_PTP_DELAY_RESP) {
    memcpy(decoded.requesting_clock, data + 44, sizeof(decoded.requesting_clock));
    decoded.requesting_port = (uint16_t)read_be(data + 52, 2);
  }
  if (type == CS_PTP_ANNOUNCE) {
    decoded.ptp_timescale = (data[7] & 0x08u) != 0;
    decoded.utc_offset_valid = (data[7] & 0x04u) != 0;
    const uint16_t offset = (uint16_t)read_be(data + 44, 2);
    decoded.utc_offset = (int16_t)(offset < 0x8000u ? (int32_t)offset : (int32_t)offset - 0x10000);
    decoded.grandmaster_clock_class = data[48];
    memcpy(decoded.grandmaster_identity, data + 53, sizeof(decoded.grandmaster_identity));
  }
  *out = decoded;
  return true;
}
