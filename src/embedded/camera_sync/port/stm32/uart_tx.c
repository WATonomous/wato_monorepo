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

#include "uart_tx.h"

#include "board.h"
#include "cs_config.h"
#include "cs_hw.h"

#define UART_BUF_SIZE 4096u  // about 3 s of reports at 115200 baud

static UART_HandleTypeDef g_huart;
static char g_buf[UART_BUF_SIZE];
static size_t g_head;  // next byte to write
static size_t g_tail;  // next byte to send
static size_t g_used;

void HAL_UART_MspInit(UART_HandleTypeDef * huart)
{
  (void)huart;
  __HAL_RCC_USART3_CLK_ENABLE();
  cs_gpio_init(CS_UART_TX_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, CS_UART_AF);
  cs_gpio_init(CS_UART_RX_PIN, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, CS_UART_AF);
}

bool cs_uart_init(void)
{
  g_huart.Instance = USART3;
  g_huart.Init.BaudRate = CS_UART_BAUD;
  g_huart.Init.WordLength = UART_WORDLENGTH_8B;
  g_huart.Init.StopBits = UART_STOPBITS_1;
  g_huart.Init.Parity = UART_PARITY_NONE;
  g_huart.Init.Mode = UART_MODE_TX_RX;
  g_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  g_huart.Init.OverSampling = UART_OVERSAMPLING_16;
  g_huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
#if defined(CS_FAMILY_H7)
  g_huart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
#endif
  g_huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  return HAL_UART_Init(&g_huart) == HAL_OK;
}

bool cs_uart_write(const char * data, size_t len)
{
  if (len > UART_BUF_SIZE - g_used) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    g_buf[g_head] = data[i];
    g_head = (g_head + 1u) % UART_BUF_SIZE;
  }
  g_used += len;
  return true;
}

void cs_uart_pump(void)
{
  while (g_used > 0u && __HAL_UART_GET_FLAG(&g_huart, UART_FLAG_TXE)) {
    g_huart.Instance->TDR = (uint8_t)g_buf[g_tail];
    g_tail = (g_tail + 1u) % UART_BUF_SIZE;
    g_used--;
  }
}
