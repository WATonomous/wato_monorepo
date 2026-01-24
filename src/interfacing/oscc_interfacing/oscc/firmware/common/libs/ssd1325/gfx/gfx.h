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

#ifndef _ADAFRUIT_GFX_H
#define _ADAFRUIT_GFX_H

#if ARDUINO >= 100
  #include "Arduino.h"
  #include "Print.h"
#else
  #include "WProgram.h"
#endif

class GFX : public Print
{
public:
  GFX(int16_t w, int16_t h);  // Constructor

  // This MUST be defined by the subclass:
  virtual void drawPixel(int16_t x, int16_t y, uint16_t color) = 0;

  // TRANSACTION API / CORE DRAW API
  // These MAY be overridden by the subclass to provide device-specific
  // optimized code.  Otherwise 'generic' versions are used.
  virtual void startWrite(void);
  virtual void writePixel(int16_t x, int16_t y, uint16_t color);
  virtual void endWrite(void);

  // CONTROL API
  // These MAY be overridden by the subclass to provide device-specific
  // optimized code.  Otherwise 'generic' versions are used.
  virtual void setRotation(uint8_t r);

  // These exist only with GFX (no subclass overrides)
  void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size),
    setCursor(int16_t x, int16_t y), setTextColor(uint16_t c), setTextColor(uint16_t c, uint16_t bg);

#if ARDUINO >= 100
  virtual size_t write(uint8_t);
#else
  virtual void write(uint8_t);
#endif

  int16_t height(void) const;
  int16_t width(void) const;

  uint8_t getRotation(void) const;

  // get current cursor position (get rotation safe maximum values, using: width() for x, height() for y)
  int16_t getCursorX(void) const;
  int16_t getCursorY(void) const;

protected:
  const int16_t WIDTH, HEIGHT;  // This is the 'raw' display w/h - never changes
  int16_t _width, _height,  // Display w/h as modified by current rotation
    cursor_x, cursor_y;
  uint16_t textcolor, textbgcolor;
  uint8_t textsize, rotation;
  boolean wrap;  // If set, 'wrap' text at right edge of display
  void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
};

#endif  // _ADAFRUIT_GFX_H
