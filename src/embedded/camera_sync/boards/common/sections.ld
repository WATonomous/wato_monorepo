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

/*
 * Output sections shared by both boards. The board script defines MEMORY and the aliases:
 *   FLASH_CODE  code and constants
 *   RAM_DATA    .data, .bss, heap and stack (CPU only)
 *   RAM_DMA     .eth_dma: Ethernet descriptors and buffers (must be reachable by the ETH DMA)
 * Symbols match the CMSIS startup files (_sidata, _sdata, _edata, _sbss, _ebss, _estack).
 */

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM_DATA) + LENGTH(RAM_DATA);
_Min_Heap_Size = 0x0;    /* no malloc in this firmware */
_Min_Stack_Size = 0x2000;

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH_CODE

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH_CODE

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH_CODE

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH_CODE
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH_CODE

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH_CODE
  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH_CODE
  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH_CODE

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM_DATA AT> FLASH_CODE

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM_DATA

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_DATA

  /* Not zeroed by the startup code: the Ethernet driver initializes what it uses. */
  .eth_dma (NOLOAD) :
  {
    . = ALIGN(32);
    *(.eth_dma)
    *(.eth_dma*)
    . = ALIGN(32);
  } >RAM_DMA

  /DISCARD/ :
  {
    libc.a(*)
    libm.a(*)
    libgcc.a(*)
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
