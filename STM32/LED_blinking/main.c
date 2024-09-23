#include "stm32f0xx.h"

// LEDs PC6 -PC9
// Button PA0

static void init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// PC6  - red LED
	GPIOC->MODER &= ~GPIO_MODER_MODER6;
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	
	GPIOC->ODR |= GPIO_ODR_6; // LED on
	GPIOC->ODR &= ~GPIO_ODR_6; // LED off
}

static void led(uint8_t state)
{
	if (state)
	{
		GPIOA->ODR |= GPIO_ODR_6; //LED on
	}
	else
	{
		GPIOA->ODR &= ~GPIO_ODR_6; //LED off
	}
}

int main(void)
{
	init();
	while(1)
	{
		//TODO
	}
}
