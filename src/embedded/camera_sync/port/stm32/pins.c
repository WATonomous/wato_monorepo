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

#include "cs/cs_pins_check.h"  // fails the build on a bad pin map
#include "cs_hw.h"

static GPIO_TypeDef * const kPorts[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK};

static void clock_enable(unsigned port)
{
  switch (port) {
    case 0:
      __HAL_RCC_GPIOA_CLK_ENABLE();
      break;
    case 1:
      __HAL_RCC_GPIOB_CLK_ENABLE();
      break;
    case 2:
      __HAL_RCC_GPIOC_CLK_ENABLE();
      break;
    case 3:
      __HAL_RCC_GPIOD_CLK_ENABLE();
      break;
    case 4:
      __HAL_RCC_GPIOE_CLK_ENABLE();
      break;
    case 5:
      __HAL_RCC_GPIOF_CLK_ENABLE();
      break;
    case 6:
      __HAL_RCC_GPIOG_CLK_ENABLE();
      break;
    case 7:
      __HAL_RCC_GPIOH_CLK_ENABLE();
      break;
    case 8:
      __HAL_RCC_GPIOI_CLK_ENABLE();
      break;
    case 9:
      __HAL_RCC_GPIOJ_CLK_ENABLE();
      break;
    case 10:
      __HAL_RCC_GPIOK_CLK_ENABLE();
      break;
    default:
      break;
  }
}

GPIO_TypeDef * cs_gpio_port(unsigned code)
{
  return kPorts[CS_PIN_PORT(code)];
}

uint16_t cs_gpio_mask(unsigned code)
{
  return (uint16_t)(1u << CS_PIN_NUM(code));
}

void cs_gpio_init(unsigned code, uint32_t mode, uint32_t pull, uint32_t speed, uint32_t alternate)
{
  if (!CS_PIN_USED(code)) {
    return;
  }
  clock_enable(CS_PIN_PORT(code));
  GPIO_InitTypeDef init = {0};
  init.Pin = cs_gpio_mask(code);
  init.Mode = mode;
  init.Pull = pull;
  init.Speed = speed;
  init.Alternate = alternate;
  HAL_GPIO_Init(cs_gpio_port(code), &init);
}

void cs_gpio_write(unsigned code, bool high)
{
  if (!CS_PIN_USED(code)) {
    return;
  }
  const uint32_t mask = cs_gpio_mask(code);
  cs_gpio_port(code)->BSRR = high ? mask : (mask << 16);
}
