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

#include <stdint.h>
#include <stdio.h>
#include <sys/timeb.h>

#include "Arduino.h"

unsigned long g_mock_arduino_millis_return;
unsigned long g_mock_arduino_micros_return;
uint8_t g_mock_arduino_digital_write_pins[100];
uint8_t g_mock_arduino_digital_write_val[100];
int g_mock_arduino_digital_write_count;
int g_mock_arduino_analog_read_return[100];
uint8_t g_mock_arduino_analog_write_pins[100];
int g_mock_arduino_analog_write_val[100];
int g_mock_arduino_analog_write_count;

unsigned long millis(void)
{
  return g_mock_arduino_millis_return;
}

unsigned long micros(void)
{
  return g_mock_arduino_micros_return;
}

void pinMode(uint8_t, uint8_t)
{}

void digitalWrite(uint8_t pin, uint8_t val)
{
  g_mock_arduino_digital_write_pins[g_mock_arduino_digital_write_count] = pin;
  g_mock_arduino_digital_write_val[g_mock_arduino_digital_write_count] = val;

  // need to keep track of successive calls to digitalWrite to be able to check
  // all of their respective values
  ++g_mock_arduino_digital_write_count;
}

int analogRead(uint8_t pin)
{
  return g_mock_arduino_analog_read_return[pin];
}

void analogWrite(uint8_t pin, int val)
{
  g_mock_arduino_analog_write_pins[g_mock_arduino_analog_write_count] = pin;
  g_mock_arduino_analog_write_val[g_mock_arduino_analog_write_count] = val;

  // need to keep track of successive calls to analogWrite to be able to check
  // all of their respective values
  ++g_mock_arduino_analog_write_count;
}

void delay(unsigned long ms)
{}

void cli(void)
{}

void sei(void)
{}

void _Serial::print(const char str[])
{
  printf("%s", str);
}

void _Serial::println(const char str[])
{
  printf("%s\n", str);
}

void _Serial::println(float f)
{
  printf("%f\n", f);
}

void _Serial::println(uint16_t d)
{
  printf("%d\n", d);
}

void _Serial::println(int16_t d)
{
  printf("%d\n", d);
}

_Serial Serial;
