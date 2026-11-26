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

// STM32 port: the only code that touches hardware. Shared by the F7 and H7 builds; the few
// differences are behind CS_FAMILY_F7 / CS_FAMILY_H7 here and in the board files.

#ifndef CS_HW_H_
#define CS_HW_H_

#include <stdbool.h>
#include <stdint.h>

#if defined(CS_FAMILY_F7)
  #include "stm32f7xx_hal.h"
#elif defined(CS_FAMILY_H7)
  #include "stm32h7xx_hal.h"
#else
  #error "Define CS_FAMILY_F7 or CS_FAMILY_H7 (the CMake board selection does this)"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  // ---- Board (boards/<board>/board_clock.c) ---------------------------------------------------
  typedef enum
  {
    CS_CLOCK_HSE = 0,  // 8 MHz from the ST-LINK MCO, crystal accuracy
    CS_CLOCK_HSI = 1,  // internal RC fallback, about 1 % accuracy
  } cs_clock_source_t;

  // System clock (PLL from HSE, falling back to HSI if HSE does not start).
  cs_clock_source_t board_clock_init(void);
  // TIM2 kernel clock in Hz (the trigger timer's tick rate).
  uint32_t board_tim2_clock_hz(void);
  // Memory/peripheral setup that must happen before anything uses DMA-visible RAM.
  void board_early_init(void);

  // ---- GPIO helpers (pins.c) ------------------------------------------------------------------
  GPIO_TypeDef * cs_gpio_port(unsigned code);
  uint16_t cs_gpio_mask(unsigned code);
  void cs_gpio_init(unsigned code, uint32_t mode, uint32_t pull, uint32_t speed, uint32_t alternate);
  void cs_gpio_write(unsigned code, bool high);

  // ---- Faults ---------------------------------------------------------------------------------
  // Drives the trigger outputs idle and lights the fault LED. Then resets the board if
  // CS_RESET_ON_FAULT is set, so triggering resumes; otherwise halts for a debugger.
  void cs_fatal(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif  // CS_HW_H_
