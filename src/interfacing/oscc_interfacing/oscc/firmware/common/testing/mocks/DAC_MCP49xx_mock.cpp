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

#include <stdio.h>

#include "DAC_MCP49xx.h"

unsigned short g_mock_dac_output_a;
unsigned short g_mock_dac_output_b;

DAC_MCP49xx::DAC_MCP49xx(Model _model, int _ss_pin, int _ldac_pin)
{}

void DAC_MCP49xx::outputA(unsigned short _out)
{
  g_mock_dac_output_a = _out;
}

void DAC_MCP49xx::outputB(unsigned short _out)
{
  g_mock_dac_output_b = _out;
}
