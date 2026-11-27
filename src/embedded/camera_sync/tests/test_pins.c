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

// Built once per board (include path picks boards/<board>/board.h). Compiling at all means the
// pin map passed the static checks; the runtime checks pin down the derived constants. The
// "bad map" cases are compile-only tests in CMakeLists.txt that must fail to build.

#include "cs/cs_pins_check.h"
#include "cs_test.h"

static void test_derived_constants(void)
{
#ifndef CS_PINS_HEADER
  CHECK_EQ_INT(CS_TRIG_PIN_MASK, 0x3FFC);  // PE2..PE13
  CHECK_EQ_INT(CS_STROBE_USED_COUNT, 12);
  CHECK_EQ_INT(CS_PPS_TIM2_CHANNEL, 1);
  const unsigned strobes[CS_STROBE_COUNT] = CS_STROBE_PIN_LIST;
  for (int i = 0; i < CS_STROBE_COUNT; ++i) {
    CHECK_EQ_INT(CS_PIN_PORT(strobes[i]), 'F' - 'A');
    CHECK_EQ_INT(CS_PIN_NUM(strobes[i]), i);
  }
#endif
}

static void test_tim2_channel_table(void)
{
  CHECK_EQ_INT(CS_TIM2_CHANNEL(CS_PIN('A', 0)), 1);
  CHECK_EQ_INT(CS_TIM2_CHANNEL(CS_PIN('A', 15)), 1);
  CHECK_EQ_INT(CS_TIM2_CHANNEL(CS_PIN('B', 3)), 2);
  CHECK_EQ_INT(CS_TIM2_CHANNEL(CS_PIN('B', 10)), 3);
  CHECK_EQ_INT(CS_TIM2_CHANNEL(CS_PIN('B', 11)), 4);
  CHECK_EQ_INT(CS_TIM2_CHANNEL(CS_PIN('E', 9)), 0);
}

int main(void)
{
  RUN(test_derived_constants);
  RUN(test_tim2_channel_table);
  TEST_MAIN_END();
}
