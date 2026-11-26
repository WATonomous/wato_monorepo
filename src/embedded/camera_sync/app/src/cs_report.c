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

#include "cs/cs_report.h"

// Small append-only writer. No printf: it is not reentrant-safe on newlib-nano and pulls in a lot
// of code. Overflow sets `ok = false` and every later append is ignored.
typedef struct
{
  char * buf;
  size_t cap;
  size_t len;
  bool ok;
} writer_t;

static void put_char(writer_t * w, char c)
{
  if (!w->ok || w->len + 1u >= w->cap) {
    w->ok = false;
    return;
  }
  w->buf[w->len++] = c;
}

static void put_str(writer_t * w, const char * s)
{
  while (*s != '\0') {
    put_char(w, *s++);
  }
}

static void put_u32(writer_t * w, uint32_t v)
{
  char tmp[10];
  int n = 0;
  do {
    tmp[n++] = (char)('0' + (v % 10u));
    v /= 10u;
  } while (v != 0u);
  while (n > 0) {
    put_char(w, tmp[--n]);
  }
}

static void put_i32(writer_t * w, int32_t v)
{
  if (v < 0) {
    put_char(w, '-');
    put_u32(w, (uint32_t)(-(int64_t)v));
  } else {
    put_u32(w, (uint32_t)v);
  }
}

static char hex_digit(uint32_t v)
{
  return (char)(v < 10u ? '0' + v : 'A' + (v - 10u));
}

static void put_hex(writer_t * w, uint32_t v, int digits)
{
  for (int i = digits - 1; i >= 0; --i) {
    put_char(w, hex_digit((v >> (4 * i)) & 0xFu));
  }
}

// Appends "*XX\r\n" and a NUL; returns the line length or 0 on overflow.
static size_t finish(writer_t * w)
{
  uint8_t sum = 0;
  for (size_t i = 0; i < w->len && w->ok; ++i) {
    sum ^= (uint8_t)w->buf[i];
  }
  put_char(w, '*');
  put_hex(w, sum, 2);
  put_char(w, '\r');
  put_char(w, '\n');
  if (!w->ok || w->len > CS_REPORT_MAX_LINE) {
    if (w->cap > 0u) {
      w->buf[0] = '\0';
    }
    return 0;
  }
  w->buf[w->len] = '\0';
  return w->len;
}

static writer_t start(char * buf, size_t cap, const char * type)
{
  writer_t w = {buf, cap, 0, buf != NULL && cap > 0u};
  put_str(&w, type);
  put_char(&w, ',');
  put_u32(&w, CS_REPORT_VERSION);
  return w;
}

size_t cs_report_boot(char * buf, size_t cap, const char * fw_version)
{
  writer_t w = start(buf, cap, "BOOT");
  put_char(&w, ',');
  // The field must be non-empty printable ASCII without commas or '*'.
  size_t n = 0;
  for (const char * c = fw_version; c != NULL && *c != '\0'; ++c) {
    if (*c >= 0x20 && *c <= 0x7E && *c != ',' && *c != '*') {
      put_char(&w, *c);
      ++n;
    }
  }
  if (n == 0u) {
    put_char(&w, '?');
  }
  return finish(&w);
}

size_t cs_report_trg(char * buf, size_t cap, const cs_pulse_t * p, bool strobes_measured, uint16_t strobe_mask)
{
  writer_t w = start(buf, cap, "TRG");
  put_char(&w, ',');
  put_u32(&w, p->seq);
  put_char(&w, ',');
  put_u32(&w, p->pps_count);
  put_char(&w, ',');
  put_u32(&w, p->pulse_idx);
  put_char(&w, ',');
  put_char(&w, p->pps_locked ? '1' : '0');
  put_char(&w, ',');
  if (strobes_measured) {
    put_hex(&w, strobe_mask & 0xFFFu, 3);
  } else {
    put_str(&w, "---");
  }
  return finish(&w);
}

const char * cs_report_state_name(cs_sync_state_t state)
{
  switch (state) {
    case CS_SYNC_LOCKED:
      return "LOCKED";
    case CS_SYNC_HOLDOVER:
      return "HOLDOVER";
    case CS_SYNC_FREERUN:
    default:
      return "FREERUN";
  }
}

size_t cs_report_stat(char * buf, size_t cap, const cs_stat_t * st)
{
  writer_t w = start(buf, cap, "STAT");
  put_char(&w, ',');
  put_u32(&w, st->uptime_s);
  put_char(&w, ',');
  put_str(&w, cs_report_state_name(st->state));
  put_char(&w, ',');
  put_i32(&w, st->pps_err_ns);
  put_char(&w, ',');
  put_u32(&w, st->pps_glitches);
  put_char(&w, ',');
  put_u32(&w, st->skipped_pulses);
  put_char(&w, ',');
  put_u32(&w, st->stray_strobes);
  put_char(&w, ',');
  put_u32(&w, st->tx_drops);
  return finish(&w);
}

size_t cs_report_record(char * buf, size_t cap, const cs_record_t * rec)
{
  switch (rec->type) {
    case CS_RECORD_TRG:
      return cs_report_trg(buf, cap, &rec->u.trg.pulse, rec->u.trg.strobes_measured, rec->u.trg.strobe_mask);
    case CS_RECORD_STAT:
      return cs_report_stat(buf, cap, &rec->u.stat);
    default:
      return 0;
  }
}
