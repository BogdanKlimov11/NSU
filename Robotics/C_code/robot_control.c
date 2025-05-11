/**
  ******************************************************************************
  * @file    robot_control.c
  * @brief   Main program for robot motor control using PWM.
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

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef g_tim1_handle;
TIM_HandleTypeDef g_tim2_handle;
TIM_HandleTypeDef g_tim4_handle;

/* Private function prototypes -----------------------------------------------*/
void system_clock_config(void);
static void gpio_init(void);
static void tim1_init(void);
static void tim2_init(void);
static void tim4_init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    system_clock_config();
    gpio_init();
    tim2_init();
    tim4_init();
    tim1_init();

    /* Initialize PWM channels */
    g_tim2_handle.Instance->CCR4 = 0;
    g_tim2_handle.Instance->CCR2 = 0;
    g_tim4_handle.Instance->CCR4 = 0;
    g_tim4_handle.Instance->CCR3 = 0;
    g_tim1_handle.Instance->CCR2 = 0;

    /* Configure GPIO pins */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_8, GPIO_PIN_SET);

    /* Start PWM */
    HAL_TIM_PWM_Start(&g_tim1_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_tim4_handle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&g_tim2_handle, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&g_tim2_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_tim4_handle, TIM_CHANNEL_4);

    /* PWM control loop */
    const uint32_t max_pwm_value = 65000;
    uint32_t current_pwm = 0;
    uint32_t previous_pwm = 0;
    const float time_constant = 1.0f;

    while (1) {
        if (current_pwm < (max_pwm_value - 5000)) {
            current_pwm = previous_pwm + (uint32_t)((float)(max_pwm_value - previous_pwm) * 0.01f / time_constant);
        } else {
            current_pwm = 0;
        }
        previous_pwm = current_pwm;

        g_tim2_handle.Instance->CCR4 = current_pwm;
        g_tim1_handle.Instance->CCR2 = current_pwm;

        HAL_Delay(10);
    }
}

/**
  * @brief  Configures the system clock.
  * @retval None
  */
void system_clock_config(void)
{
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc_init.HSIState = RCC_HSI_ON;
    osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc_init.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
        Error_Handler();
    }

    clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
    clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  Initializes TIM1 for PWM.
  * @retval None
  */
static void tim1_init(void)
{
    TIM_ClockConfigTypeDef clock_config = {0};
    TIM_MasterConfigTypeDef master_config = {0};
    TIM_OC_InitTypeDef oc_config = {0};
    TIM_BreakDeadTimeConfigTypeDef break_config = {0};

    g_tim1_handle.Instance = TIM1;
    g_tim1_handle.Init.Prescaler = 0;
    g_tim1_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim1_handle.Init.Period = 65535;
    g_tim1_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_tim1_handle.Init.RepetitionCounter = 0;
    g_tim1_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&g_tim1_handle) != HAL_OK) {
        Error_Handler();
    }

    clock_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&g_tim1_handle, &clock_config) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&g_tim1_handle) != HAL_OK) {
        Error_Handler();
    }

    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&g_tim1_handle, &master_config) != HAL_OK) {
        Error_Handler();
    }

    oc_config.OCMode = TIM_OCMODE_PWM1;
    oc_config.Pulse = 0;
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc_config.OCFastMode = TIM_OCFAST_DISABLE;
    oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&g_tim1_handle, &oc_config, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    break_config.OffStateRunMode = TIM_OSSR_DISABLE;
    break_config.OffStateIDLEMode = TIM_OSSI_DISABLE;
    break_config.LockLevel = TIM_LOCKLEVEL_OFF;
    break_config.DeadTime = 0;
    break_config.BreakState = TIM_BREAK_DISABLE;
    break_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    break_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&g_tim1_handle, &break_config) != HAL_OK) {
        Error_Handler();
    }

    hal_tim_msp_post_init(&g_tim1_handle);
}

/**
  * @brief  Initializes TIM2 for PWM.
  * @retval None
  */
static void tim2_init(void)
{
    TIM_ClockConfigTypeDef clock_config = {0};
    TIM_MasterConfigTypeDef master_config = {0};
    TIM_OC_InitTypeDef oc_config = {0};

    g_tim2_handle.Instance = TIM2;
    g_tim2_handle.Init.Prescaler = 0;
    g_tim2_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim2_handle.Init.Period = 65535;
    g_tim2_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_tim2_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&g_tim2_handle) != HAL_OK) {
        Error_Handler();
    }

    clock_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&g_tim2_handle, &clock_config) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&g_tim2_handle) != HAL_OK) {
        Error_Handler();
    }

    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&g_tim2_handle, &master_config) != HAL_OK) {
        Error_Handler();
    }

    oc_config.OCMode = TIM_OCMODE_PWM1;
    oc_config.Pulse = 0;
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc_config.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&g_tim2_handle, &oc_config, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&g_tim2_handle, &oc_config, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    hal_tim_msp_post_init(&g_tim2_handle);
}

/**
  * @brief  Initializes TIM4.
  * @retval None
  */
static void tim4_init(void)
{
    TIM_ClockConfigTypeDef clock_config = {0};
    TIM_MasterConfigTypeDef master_config = {0};

    g_tim4_handle.Instance = TIM4;
    g_tim4_handle.Init.Prescaler = 0;
    g_tim4_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim4_handle.Init.Period = 65535;
    g_tim4_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_tim4_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&g_tim4_handle) != HAL_OK) {
        Error_Handler();
    }

    clock_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&g_tim4_handle, &clock_config) != HAL_OK) {
        Error_Handler();
    }

    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&g_tim4_handle, &master_config) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  Initializes GPIO pins.
  * @retval None
  */
static void gpio_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

    gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_8;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    gpio_init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio_init);
}

/**
  * @brief  Error handler.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
