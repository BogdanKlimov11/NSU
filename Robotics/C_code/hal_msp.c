/**
  ******************************************************************************
  * @file    hal_msp.c
  * @brief   MSP initialization and de-initialization for STM32F1xx.
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

/* Private function prototypes -----------------------------------------------*/
void hal_tim_msp_post_init(TIM_HandleTypeDef *htim);

/**
  * @brief  Initializes the global MSP.
  * @retval None
  */
void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /** Disable JTAG-DP and SW-DP */
    __HAL_AFIO_REMAP_SWJ_DISABLE();
}

/**
  * @brief  Initializes TIM_Base MSP.
  * @param  htim_base TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    } else if (htim_base->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
    } else if (htim_base->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }
}

/**
  * @brief  Post-initialization for TIM MSP.
  * @param  htim TIM handle pointer
  * @retval None
  */
void hal_tim_msp_post_init(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio_init = {0};

    if (htim->Instance == TIM1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /** TIM1 GPIO Configuration: PA9 -> TIM1_CH2 */
        gpio_init.Pin = GPIO_PIN_9;
        gpio_init.Mode = GPIO_MODE_AF_PP;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &gpio_init);
    } else if (htim->Instance == TIM2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /** TIM2 GPIO Configuration: PA1 -> TIM2_CH2, PA3 -> TIM2_CH4 */
        gpio_init.Pin = GPIO_PIN_1 | GPIO_PIN_3;
        gpio_init.Mode = GPIO_MODE_AF_PP;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &gpio_init);
    }
}

/**
  * @brief  De-initializes TIM_Base MSP.
  * @param  htim_base TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_DISABLE();
    } else if (htim_base->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();
    } else if (htim_base->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_DISABLE();
    }
}
