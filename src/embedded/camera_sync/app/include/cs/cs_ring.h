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

// Lock-free single-producer / single-consumer queue of report records.
// Producer: the timer interrupt. Consumer: the main loop. Neither side ever blocks or disables
// interrupts; a full queue drops the new record and the caller counts it.

#ifndef CS_RING_H_
#define CS_RING_H_

#include <stdbool.h>
#include <stdint.h>

#include "cs/cs_report.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CS_RING_SIZE 64u  // power of two
  _Static_assert((CS_RING_SIZE & (CS_RING_SIZE - 1u)) == 0u, "CS_RING_SIZE must be a power of two");

  typedef struct
  {
    cs_record_t items[CS_RING_SIZE];
    uint32_t head;  // written by the producer only
    uint32_t tail;  // written by the consumer only
  } cs_ring_t;

  static inline void cs_ring_init(cs_ring_t * r)
  {
    r->head = 0;
    r->tail = 0;
  }

  static inline bool cs_ring_push(cs_ring_t * r, const cs_record_t * rec)
  {
    const uint32_t head = r->head;
    const uint32_t tail = __atomic_load_n(&r->tail, __ATOMIC_ACQUIRE);
    if (head - tail >= CS_RING_SIZE) {
      return false;
    }
    r->items[head & (CS_RING_SIZE - 1u)] = *rec;
    __atomic_store_n(&r->head, head + 1u, __ATOMIC_RELEASE);
    return true;
  }

  static inline bool cs_ring_pop(cs_ring_t * r, cs_record_t * out)
  {
    const uint32_t tail = r->tail;
    const uint32_t head = __atomic_load_n(&r->head, __ATOMIC_ACQUIRE);
    if (head == tail) {
      return false;
    }
    *out = r->items[tail & (CS_RING_SIZE - 1u)];
    __atomic_store_n(&r->tail, tail + 1u, __ATOMIC_RELEASE);
    return true;
  }

#ifdef __cplusplus
}
#endif

#endif  // CS_RING_H_
