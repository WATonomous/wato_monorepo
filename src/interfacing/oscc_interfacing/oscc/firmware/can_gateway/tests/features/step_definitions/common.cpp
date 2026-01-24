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

#include <cgreen/cgreen.h>
#include <cgreen/mocks.h>
#include <sys/timeb.h>
#include <unistd.h>

#include "Arduino.h"
#include "communications.h"
#include "globals.h"
#include "mcp_can.h"
#include "oscc_can.h"
#include "vehicles.h"

#include <cucumber-cpp/autodetect.hpp>

using namespace cgreen;

extern uint8_t g_mock_mcp_can_check_receive_return;
extern uint32_t g_mock_mcp_can_read_msg_buf_id;
extern uint32_t g_mock_mcp_can_send_msg_buf_id;

// return to known state before every scenario
BEFORE()
{
  g_mock_mcp_can_check_receive_return = UINT8_MAX;
  g_mock_mcp_can_read_msg_buf_id = UINT32_MAX;
  g_mock_mcp_can_send_msg_buf_id = UINT32_MAX;
}
