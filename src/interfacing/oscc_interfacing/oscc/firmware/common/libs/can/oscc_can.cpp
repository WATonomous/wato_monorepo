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

/**
 * @file oscc_can.cpp
 *
 */

#include "oscc_can.h"

#include <string.h>

#include "debug.h"
#include "mcp_can.h"

void init_can(MCP_CAN & can)
{
  while (can.begin(CAN_BAUD) != CAN_OK) {
    delay(CAN_INIT_RETRY_DELAY);
    DEBUG_PRINTLN("init_can: retrying");
  }

  DEBUG_PRINTLN("init_can: pass");
}

can_status_t check_for_rx_frame(MCP_CAN & can, can_frame_s * const frame)
{
  can_status_t ret = CAN_RX_FRAME_UNKNOWN;

  if (frame != NULL) {
    memset(frame, 0, sizeof(*frame));

    cli();

    int got_message = can.readMsgBufID((uint32_t *)&frame->id, (uint8_t *)&frame->dlc, (uint8_t *)frame->data);

    if (got_message == CAN_OK) {
      frame->timestamp = millis();

      ret = CAN_RX_FRAME_AVAILABLE;
    } else {
      ret = CAN_RX_FRAME_UNAVAILABLE;
    }

    sei();
  }

  return ret;
}
