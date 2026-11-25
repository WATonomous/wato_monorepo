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

// NUCLEO-F767ZI clocks: SYSCLK 216 MHz, APB1 54 MHz (TIM2 108 MHz), APB2 108 MHz.
// Source: the ST-LINK's 8 MHz MCO output on PH0 (HSE bypass, crystal accuracy). If it does not
// start (solder bridge SB148 open), the 16 MHz HSI is used instead (about 1 %).

#include "cs_hw.h"

static bool pll_config(uint32_t source)
{
  RCC_OscInitTypeDef osc = {0};
  if (source == RCC_PLLSOURCE_HSE) {
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_BYPASS;
    osc.PLL.PLLM = 4;  // 8 MHz / 4 = 2 MHz
  } else {
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLM = 8;  // 16 MHz / 8 = 2 MHz
  }
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = source;
  osc.PLL.PLLN = 216;  // VCO 432 MHz
  osc.PLL.PLLP = RCC_PLLP_DIV2;  // 216 MHz
  osc.PLL.PLLQ = 9;
  osc.PLL.PLLR = 7;
  return HAL_RCC_OscConfig(&osc) == HAL_OK;
}

cs_clock_source_t board_clock_init(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  cs_clock_source_t used = CS_CLOCK_HSE;
  if (!pll_config(RCC_PLLSOURCE_HSE)) {
    used = CS_CLOCK_HSI;
    RCC_OscInitTypeDef off = {0};
    off.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    off.HSEState = RCC_HSE_OFF;
    off.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&off);
    if (!pll_config(RCC_PLLSOURCE_HSI)) {
      cs_fatal();
    }
  }
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    cs_fatal();
  }

  RCC_ClkInitTypeDef clk = {0};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4;
  clk.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_7) != HAL_OK) {
    cs_fatal();
  }
  return used;
}

uint32_t board_tim2_clock_hz(void)
{
  // APB1 timers run at twice PCLK1 when the APB1 prescaler is not 1.
  const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  return (RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1 ? pclk1 : 2u * pclk1;
}

void board_early_init(void)
{}
