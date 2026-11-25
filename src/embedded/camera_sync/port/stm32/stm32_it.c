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

// Core exception and EXTI handlers. TIM2_IRQHandler is in trigger.c.

#include "cs_hw.h"
#include "trigger.h"

void NMI_Handler(void)
{
  cs_fatal();
}

void HardFault_Handler(void)
{
  cs_fatal();
}

void MemManage_Handler(void)
{
  cs_fatal();
}

void BusFault_Handler(void)
{
  cs_fatal();
}

void UsageFault_Handler(void)
{
  cs_fatal();
}

void SVC_Handler(void)
{}

void DebugMon_Handler(void)
{}

void PendSV_Handler(void)
{}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

// Strobe inputs: every EXTI vector does the same work (only lines in use are enabled).
void EXTI0_IRQHandler(void)
{
  cs_trigger_exti_irq();
}

void EXTI1_IRQHandler(void)
{
  cs_trigger_exti_irq();
}

void EXTI2_IRQHandler(void)
{
  cs_trigger_exti_irq();
}

void EXTI3_IRQHandler(void)
{
  cs_trigger_exti_irq();
}

void EXTI4_IRQHandler(void)
{
  cs_trigger_exti_irq();
}

void EXTI9_5_IRQHandler(void)
{
  cs_trigger_exti_irq();
}

void EXTI15_10_IRQHandler(void)
{
  cs_trigger_exti_irq();
}
