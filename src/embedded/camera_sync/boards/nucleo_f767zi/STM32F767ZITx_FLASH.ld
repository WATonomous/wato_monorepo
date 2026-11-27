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

/* STM32F767ZI: 2 MB flash; 512 KB RAM = DTCM 128 KB + SRAM1 368 KB + SRAM2 16 KB (contiguous). */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 496K  /* DTCM + SRAM1 */
  SRAM2 (xrw) : ORIGIN = 0x2007C000, LENGTH = 16K   /* Ethernet DMA buffers */
}

REGION_ALIAS("FLASH_CODE", FLASH);
REGION_ALIAS("RAM_DATA", RAM);
REGION_ALIAS("RAM_DMA", SRAM2);

INCLUDE sections.ld
