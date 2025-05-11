/**
  ******************************************************************************
  * @file    system_init.c
  * @brief   CMSIS Cortex-M3 system initialization for STM32F1xx.
  * @author  STMicroelectronics
  * @date    2025
  *
  * @attention
  * Copyright (c) 2025 STMicroelectronics. All rights reserved.
  * Licensed under BSD 3-Clause license. See opensource.org/licenses/BSD-3-Clause.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"

/* Private defines -----------------------------------------------------------*/
#ifndef HSE_VALUE
#define HSE_VALUE 8000000U /**< External oscillator frequency in Hz */
#endif

#ifndef HSI_VALUE
#define HSI_VALUE 8000000U /**< Internal oscillator frequency in Hz */
#endif

/* Private variables ---------------------------------------------------------*/
uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U] = {0, 0, 0, 0, 1, 2, 3, 4};

/* Functions -----------------------------------------------------------------*/
/**
  * @brief  Initializes the microcontroller system.
  * @retval None
  */
void SystemInit(void)
{
#if defined(USER_VECT_TAB_ADDRESS)
    SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;
#endif
}

/**
  * @brief  Updates SystemCoreClock variable based on clock settings.
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
    uint32_t tmp = 0U, pllmull = 0U, pllsource = 0U;

    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp) {
        case 0x00U: /* HSI used as system clock */
            SystemCoreClock = HSI_VALUE;
            break;
        case 0x04U: /* HSE used as system clock */
            SystemCoreClock = HSE_VALUE;
            break;
        case 0x08U: /* PLL used as system clock */
            pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
            pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

            pllmull = (pllmull >> 18U) + 2U;

            if (pllsource == 0x00U) {
                /* HSI oscillator clock divided by 2 */
                SystemCoreClock = (HSI_VALUE >> 1U) * pllmull;
            } else {
                if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != (uint32_t)RESET) {
                    /* HSE oscillator clock divided by 2 */
                    SystemCoreClock = (HSE_VALUE >> 1U) * pllmull;
                } else {
                    SystemCoreClock = HSE_VALUE * pllmull;
                }
            }
            break;
        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }

    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
    SystemCoreClock >>= tmp;
}
