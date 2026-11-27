/*
 * Copyright (c) 2025-present WATonomous. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* STM32H753ZI: 2 MB flash; DTCM 128 KB (CPU only, zero wait), AXI SRAM 512 KB, D2 SRAM 288 KB. */

MEMORY
{
  FLASH (rx)    : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1 (xrw)  : ORIGIN = 0x24000000, LENGTH = 512K
  RAM_D2 (xrw)  : ORIGIN = 0x30000000, LENGTH = 288K
}

REGION_ALIAS("FLASH_CODE", FLASH);
REGION_ALIAS("RAM_DATA", DTCMRAM);  /* the Ethernet DMA cannot reach the DTCM, see RAM_DMA */
REGION_ALIAS("RAM_DMA", RAM_D2);

INCLUDE sections.ld
