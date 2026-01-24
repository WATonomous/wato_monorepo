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

#ifndef _OSCC_TEST_MOCK_DAC_MCP49XX_H_
#define _OSCC_TEST_MOCK_DAC_MCP49XX_H_

class DAC_MCP49xx
{
public:
  enum Model
  {
    MCP4901 = 1, /* single, 8-bit */
    MCP4911, /* single, 10-bit */
    MCP4921, /* single, 12-bit */
    MCP4902, /* dual, 8-bit */
    MCP4912, /* dual, 10-bit */
    MCP4922 /* dual, 12-bit */
  };

  DAC_MCP49xx(Model _model, int _ss_pin, int _ldac_pin = -1);

  void outputA(unsigned short _out);
  void outputB(unsigned short _out);
};

#endif
