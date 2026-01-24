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

#include <Arduino.h>

class SSD1325
{
public:
  SSD1325(int8_t SID, int8_t SCLK, int8_t DC, int8_t RST, int8_t CS)
  {}

  void begin(void);
  void eraseBuffer(void);
  void sendBuffer(void);

private:
  int8_t sid;
  int8_t sclk;
  int8_t dc;
  int8_t rst;
  int8_t cs;

  void startSendCommand(void);
  void stopSendCommand(void);
  void startSendData(void);
  void stopSendData(void);

  void drawPixel(int16_t x, int16_t y, uint16_t color);
};
