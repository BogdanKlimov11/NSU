#include "stm32f0xx.h"

void init(void);

void init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOC -> MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
	GPIOC -> MODER |= (GPIO_MODER_MODER6_0| GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0| GPIO_MODER_MODER9_0);

	GPIOC->ODR |= GPIO_ODR_6;

	GPIOA->MODER &=~ GPIO_MODER_MODER15;
	GPIOC->MODER &=~ GPIO_MODER_MODER12;
	GPIOA->MODER &=~ GPIO_MODER_MODER4;
	GPIOA->MODER &=~ GPIO_MODER_MODER5;
	GPIOA->MODER |= GPIO_MODER_MODER15_0;
	GPIOC->MODER |= GPIO_MODER_MODER12_0;
	//GPIOA->MODER |= GPIO_MODER_MODER4_1;
	//GPIOC->MODER |= GPIO_MODER_MODER5_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_1;
	SystemCoreClockUpdate(); // System_Config(SystemCoreClock)
	SysTick->LOAD = SystemCoreClock / 1000 - 1; // 1 ms
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}


//static uint8_t systick_flag_check(void){
//	return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1 : 0;
//}


static uint8_t button_read1(void){
	return (GPIOA->IDR & GPIO_IDR_4) ? 1 : 0;
}


static uint8_t button_read2(void){
	return (GPIOA->IDR & GPIO_IDR_5) ? 1 : 0;
}


int main(){
	init();
	while(1){
		GPIOA->BSRR = GPIO_BSRR_BS_15;
		if(button_read1()){
			GPIOC->BSRR = GPIO_BSRR_BS_6;
		}
		else{
			GPIOC->BRR=GPIO_BRR_BR_6;
		}
		if(button_read2()){
			GPIOC->BSRR = GPIO_BSRR_BS_7;
		}
		else{
			GPIOC->BRR=GPIO_BRR_BR_7;
		}
		
		// change channel
		GPIOA->BRR = GPIO_BRR_BR_15;
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		GPIOC->BSRR = GPIO_BSRR_BS_12;
		
		if(button_read1()){
			GPIOC->BSRR = GPIO_BSRR_BS_9;
		}
		else{
			GPIOC->BRR=GPIO_BRR_BR_9;
		}
		if(button_read2()){
			GPIOC->BSRR = GPIO_BSRR_BS_8;
		}
		else{
			GPIOC->BRR=GPIO_BRR_BR_8;
		}
		GPIOC->BRR = GPIO_BRR_BR_12;
	}
}
