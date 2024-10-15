#include "stm32f0xx.h"
#include "leds.h"
#include "buttons.h"

uint8_t systick_flat_check(void){
	return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1: 0;
}


int delay(void){
	if(systick_flat_check() == 1){
		return(1);
	}
	else {
		return(0);
	}
}


void light(uint8_t state, struct buttons* buttons){
	if(state){
		if(buttons->first == 1){
			//GPIOC->ODR &= ~GPIO_ODR_6;
			GPIOC->BRR = GPIO_BRR_BR_6;
			//GPIOC->ODR |= GPIO_ODR_8;
			GPIOC->BSRR = GPIO_BSRR_BS_8;
			buttons->first = 0;
			buttons->second = 1;
			return;
			//GPIOC->BSRR = (state) ? CPIO_BSRR_BS_6 : GPIO_BSRR_BR_6; // for general purpose ports only
		}
		if(buttons->second == 1){
			//GPIOC->ODR &= ~GPIO_ODR_8;
			GPIOC->BRR = GPIO_BRR_BR_8;
			//GPIOC->ODR |= GPIO_ODR_7; 
			GPIOC->BSRR = GPIO_BSRR_BS_7;
			buttons->second = 0;
			buttons->third = 1;
			return;
		}
		if(buttons->third == 1){
			//GPIOC->ODR &= ~GPIO_ODR_7;
			GPIOC->BRR = GPIO_BRR_BR_7;
			//GPIOC->ODR |= GPIO_ODR_9; 
			GPIOC->BSRR = GPIO_BSRR_BS_9;
			buttons->third = 0;
			buttons->four = 1;
			return;
    }
		if(buttons->four == 1){
			//GPIOC->ODR &= ~GPIO_ODR_9;
			GPIOC->BRR = GPIO_BRR_BR_9;
			//GPIOC->ODR |= GPIO_ODR_6; 
			GPIOC->BSRR = GPIO_BSRR_BS_6;
			buttons->four = 0;
			buttons->first = 1;
			return;
    }
	}
	else{
		if(buttons->first == 1){
			//GPIOC->ODR &= ~GPIO_ODR_6;
			GPIOC->BRR = GPIO_BRR_BR_6;
			//GPIOC->ODR |= GPIO_ODR_9; 
			GPIOC->BSRR = GPIO_BSRR_BS_9;
			buttons->first = 0;
			buttons->four = 1;
			return;
		}
		if(buttons->second == 1){
			//GPIOC->ODR &= ~GPIO_ODR_8;
			GPIOC->BRR = GPIO_BRR_BR_8;
			//GPIOC->ODR |= GPIO_ODR_6; 
			GPIOC->BSRR = GPIO_BSRR_BS_6;
			buttons->second = 0;
			buttons->first = 1;
			return;
		}
		if(buttons->third == 1){
			//GPIOC->ODR &= ~GPIO_ODR_7;
			GPIOC->BRR = GPIO_BRR_BR_7;
			//GPIOC->ODR |= GPIO_ODR_8; 
			GPIOC->BSRR = GPIO_BSRR_BS_8;
			buttons->third = 0;
			buttons->second = 1;
			return;
		}
		if(buttons->four == 1){
			//GPIOC->ODR &= ~GPIO_ODR_9;
			GPIOC->BRR = GPIO_BRR_BR_9;
			//GPIOC->ODR |= GPIO_ODR_7; 
			GPIOC->BSRR = GPIO_BSRR_BS_7;
			buttons->four = 0;
			buttons->third = 1;
			return;
		}
	}
}
