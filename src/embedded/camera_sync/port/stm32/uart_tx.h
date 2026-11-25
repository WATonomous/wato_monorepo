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

// Report lines over the ST-LINK virtual COM port (USART3, 8N1). Main loop only: lines are queued
// in a buffer and fed to the transmit register by cs_uart_pump(), so nothing ever waits.

#ifndef UART_TX_H_
#define UART_TX_H_

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

  bool cs_uart_init(void);
  // Queues a whole line, or nothing if it does not fit (returns false).
  bool cs_uart_write(const char * data, size_t len);
  void cs_uart_pump(void);

#ifdef __cplusplus
}
#endif

#endif  // UART_TX_H_
