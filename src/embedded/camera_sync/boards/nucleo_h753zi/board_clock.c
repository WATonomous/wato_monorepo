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

// NUCLEO-H753ZI clocks: SYSCLK 400 MHz, AHB 200 MHz, APB1 100 MHz (TIM2 200 MHz).
// Source: the ST-LINK's 8 MHz MCO output on PH0 (HSE bypass, crystal accuracy). If it does not
// start, the 64 MHz HSI is used instead (about 1 %). Supply: internal LDO (board default).

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
    osc.HSIState = RCC_HSI_DIV1;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLM = 32;  // 64 MHz / 32 = 2 MHz
  }
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = source;
  osc.PLL.PLLN = 400;  // VCO 800 MHz
  osc.PLL.PLLP = 2;  // 400 MHz
  osc.PLL.PLLQ = 4;
  osc.PLL.PLLR = 2;
  osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;  // 2-4 MHz input
  osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  osc.PLL.PLLFRACN = 0;
  return HAL_RCC_OscConfig(&osc) == HAL_OK;
}

cs_clock_source_t board_clock_init(void)
{
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

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

  RCC_ClkInitTypeDef clk = {0};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
  clk.AHBCLKDivider = RCC_HCLK_DIV2;
  clk.APB3CLKDivider = RCC_APB3_DIV2;
  clk.APB1CLKDivider = RCC_APB1_DIV2;
  clk.APB2CLKDivider = RCC_APB2_DIV2;
  clk.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
    cs_fatal();
  }
  return used;
}

uint32_t board_tim2_clock_hz(void)
{
  // APB1 timers run at twice PCLK1 when the APB1 prescaler is not 1 (TIMPRE = 0).
  const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  return (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) == RCC_D2CFGR_D2PPRE1_DIV1 ? pclk1 : 2u * pclk1;
}

void board_early_init(void)
{
  // D2 SRAM holds the Ethernet DMA buffers (the DMA cannot reach the DTCM).
  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();
}
