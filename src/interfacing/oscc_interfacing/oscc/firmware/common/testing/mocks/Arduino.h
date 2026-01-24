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

#ifndef _OSCC_TEST_MOCK_ARDUINO_H_
#define _OSCC_TEST_MOCK_ARDUINO_H_

#include <stdint.h>
#include <stdio.h>

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

unsigned long millis(void);

unsigned long micros(void);

void pinMode(uint8_t, uint8_t);

void digitalWrite(uint8_t pin, uint8_t val);

int analogRead(uint8_t pin);

void analogWrite(uint8_t pin, int val);

void delay(unsigned long ms);

void sei();

void cli();

class _Serial
{
public:
  void begin(unsigned long);
  void print(const char[]);
  void println(const char[]);
  void println(float f);
  void println(uint16_t d);
  void println(int16_t d);
};

extern _Serial Serial;

#endif
