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

#ifndef _OSCC_TEST_MOCK_MCP_CAN_H_
#define _OSCC_TEST_MOCK_MCP_CAN_H_

#include <stdint.h>

#define CAN_500KBPS 15

#define CAN_OK (0)
#define CAN_FAILINIT (1)
#define CAN_FAILTX (2)
#define CAN_MSGAVAIL (3)
#define CAN_NOMSG (4)
#define CAN_CTRLERROR (5)
#define CAN_GETTXBFTIMEOUT (6)
#define CAN_SENDMSGTIMEOUT (7)
#define CAN_FAIL (0xff)

class MCP_CAN
{
public:
  MCP_CAN(uint8_t _CS);
  uint8_t begin(uint8_t speedset);
  uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t * buf);
  uint8_t readMsgBufID(uint32_t * ID, uint8_t * len, uint8_t * buf);
};

#endif
