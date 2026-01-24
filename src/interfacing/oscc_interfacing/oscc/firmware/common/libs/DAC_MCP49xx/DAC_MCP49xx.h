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

#ifndef _DAC_MCP49xx_H
#define _DAC_MCP49xx_H

#include <Arduino.h>
#include <inttypes.h>
#include <SPI.h>

#ifndef _SPI_H_INCLUDED
  #error Please include SPI.h before DAC_MCP49xx.h!
#endif

// Microchip MCP4901/MCP4911/MCP4921/MCP4902/MCP4912/MCP4922 DAC driver
// Thomas Backman, 2012

class DAC_MCP49xx
{
public:
  // These are the DAC models we support
  enum Model
  {
    MCP4901 = 1, /* single, 8-bit */
    MCP4911, /* single, 10-bit */
    MCP4921, /* single, 12-bit */
    MCP4902, /* dual, 8-bit */
    MCP4912, /* dual, 10-bit */
    MCP4922 /* dual, 12-bit */
  };

  enum Channel
  {
    CHANNEL_A = 0,
    CHANNEL_B = 1
  };

  DAC_MCP49xx(Model _model, int _ss_pin, int _ldac_pin = -1);
  void setBuffer(boolean _buffer);
  void setPortWrite(boolean _port_write);
  boolean setGain(int _gain);
  boolean setSPIDivider(int _spi_divider);
  void shutdown(void);

  /* All of these call _output() internally */
  void output(unsigned short _out);
  void outputA(unsigned short _out); /* same as output(), but having A/B makes more sense for dual DACs */
  void outputB(unsigned short _out);
  void output2(unsigned short _out, unsigned short _out2);  // For MCP49x2

  void latch(void);  // Actually change the output, if the LDAC pin isn't shorted to ground
  boolean setAutomaticallyLatchDual(bool _latch);

private:
  void _output(unsigned short _out, Channel _chan);
  int ss_pin;
  int LDAC_pin;
  int bitwidth;
  boolean bufferVref;
  boolean gain2x; /* false -> 1x, true -> 2x */
  boolean port_write; /* use optimized port writes? won't work everywhere! */
  int spi_divider;
  boolean automaticallyLatchDual; /* call latch() automatically after output2() has been called? */
};

#endif
