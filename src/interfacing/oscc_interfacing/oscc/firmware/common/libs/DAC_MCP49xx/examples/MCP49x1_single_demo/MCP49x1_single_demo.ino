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

//
// Example for the MCP49x1 *single* DACs
// For the dual MCP49x2 series, see the other bundled example sketch.
//

#include <DAC_MCP49xx.h>
#include <SPI.h>  // Remember this line!

// The Arduino pin used for the slave select / chip select
#define SS_PIN 10

// Set up the DAC.
// First argument: DAC model (MCP4901, MCP4911, MCP4921)
// Second argument: SS pin (10 is preferred)
// (The third argument, the LDAC pin, can be left out if not used)
DAC_MCP49xx dac(DAC_MCP49xx::MCP4901, SS_PIN);

void setup()
{
  // Set the SPI frequency to 1 MHz (on 16 MHz Arduinos), to be safe.
  // DIV2 = 8 MHz works for me, though, even on a breadboard.
  // This is not strictly required, as there is a default setting.
  dac.setSPIDivider(SPI_CLOCK_DIV16);

  // Use "port writes", see the manual page. In short, if you use pin 10 for
  // SS (and pin 7 for LDAC, if used), this is much faster.
  // Also not strictly required (no setup() code is needed at all).
  dac.setPortWrite(true);
}

// Output something slow enough that a multimeter can pick it up.
// For MCP4901, use values below (but including) 255.
// For MCP4911, use values below (but including) 1023.
// For MCP4921, use values below (but including) 4095.
void loop()
{
  dac.output(255);
  delay(2500);
  dac.output(0);
  delay(2500);
}
