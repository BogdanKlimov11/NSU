/**
  ******************************************************************************
  * @file    interrupt_handlers.c
  * @brief   Interrupt service routines for STM32F1xx.
  * @author  STMicroelectronics
  * @date    2025
  *
  * @attention
  * Copyright (c) 2025 STMicroelectronics. All rights reserved.
  * Licensed under BSD 3-Clause license. See opensource.org/licenses/BSD-3-Clause.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "interrupt_handlers.h"

/**
  * @brief  Handles non-maskable interrupt.
  * @retval None
  */
void NMI_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief  Handles hard fault interrupt.
  * @retval None
  */
void HardFault_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief  Handles memory management fault.
  * @retval None
  */
void MemManage_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief  Handles prefetch or memory access fault.
  * @retval None
  */
void BusFault_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief  Handles undefined instruction or illegal state.
  * @retval None
  */
void UsageFault_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief  Handles system service call via SWI instruction.
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  Handles debug monitor.
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  Handles pendable request for system service.
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  Handles system tick timer.
  * @retval None
  */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
