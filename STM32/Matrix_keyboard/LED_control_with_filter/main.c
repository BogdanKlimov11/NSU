#include "button.h"
#include "stm32f0xx.h"

void init(void);

uint8_t systick_flag_check(void);

void CaseLedInvert(uint8_t state, int8_t num);

uint8_t button_0_read(void);
uint8_t button_1_read(void);
uint8_t button_2_read(void);
uint8_t button_3_read(void);
uint8_t button_4_read(void);

void AllLedInvert(uint8_t state);

void init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	
	GPIOA->MODER &= ~(GPIO_MODER_MODER0|GPIO_MODER_MODER4|GPIO_MODER_MODER5);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4|GPIO_PUPDR_PUPDR5);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1|GPIO_PUPDR_PUPDR5_1;
	GPIOC->MODER |= GPIO_MODER_MODER12_0;
	GPIOA->MODER |= GPIO_MODER_MODER15_0;
	
	GPIOC->MODER &= ~(GPIO_MODER_MODER6|GPIO_MODER_MODER7|GPIO_MODER_MODER8|GPIO_MODER_MODER9);
	GPIOC->MODER |= GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0;
	
	SystemCoreClockUpdate();
	SysTick->LOAD = SystemCoreClock / 1000 - 1; // 1 ms
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}

uint8_t systick_flag_check(void){
	return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1 : 0;
}

void CaseLedInvert(uint8_t state, int8_t num){
	if (state){
		switch (num){
			case 1:
				GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_6) == GPIO_ODR_6) ? GPIO_BSRR_BR_6 : GPIO_BSRR_BS_6;
				break;
			case 2:
				GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_9) == GPIO_ODR_9) ? GPIO_BSRR_BR_9 : GPIO_BSRR_BS_9;
				break;
			case 3:
				GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_7) == GPIO_ODR_7) ? GPIO_BSRR_BR_7 : GPIO_BSRR_BS_7;
				break;
			case 4:
				GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_8) == GPIO_ODR_8) ? GPIO_BSRR_BR_8 : GPIO_BSRR_BS_8;
				break;
			default:
				break;
		}
	}
}


uint8_t button_0_read(void){
	return (GPIO_IDR_0 & GPIOA->IDR) ? 1 : 0;
}


uint8_t button_1_read(void){
	//GPIOA->ODR |= GPIO_ODR_15;
	return (GPIO_IDR_4 & GPIOA->IDR) ? 1 : 0;
	//GPIOA->ODR &= ~GPIO_ODR_15;
	//return a;
}


uint8_t button_2_read(void){
	//GPIOA->ODR |= GPIO_ODR_15;
	return (GPIO_IDR_5 & GPIOA->IDR) ? 1 : 0;
	//GPIOA->ODR &= ~GPIO_ODR_15;
	//return a;
}


uint8_t button_3_read(void){
	//GPIOC->ODR |= GPIO_ODR_12;
	return (GPIO_IDR_4 & GPIOA->IDR) ? 1 : 0;
	//GPIOC->ODR &= ~GPIO_ODR_12;
	//return a;
}


uint8_t button_4_read(void){
	//GPIOC->ODR |= GPIO_ODR_12;
	return (GPIO_IDR_5 & GPIOA->IDR) ? 1 : 0;
	//GPIOC->ODR &= ~GPIO_ODR_12;
	//return a;
}


void AllLedInvert(uint8_t state){
	CaseLedInvert(state, 1);
	CaseLedInvert(state, 2);
	CaseLedInvert(state, 3);
	CaseLedInvert(state, 4);
}

int main(void){
	uint8_t sysTick = 0;
	init();
	ConfButton button0 = InitButton(button_0_read);
	ConfButton button1 = InitButton(button_1_read);
	ConfButton button2 = InitButton(button_2_read);
	ConfButton button3 = InitButton(button_3_read);
	ConfButton button4 = InitButton(button_4_read);
	while(1){
		sysTick = systick_flag_check();
		if (sysTick==1){
			HandlerButton(&button0);
			
			GPIOA->ODR |= GPIO_ODR_15;
			HandlerButton(&button1);
			HandlerButton(&button2);
			GPIOA->ODR &= ~GPIO_ODR_15;
			
			GPIOC->ODR |= GPIO_ODR_12;
			HandlerButton(&button3);
			HandlerButton(&button4);
			GPIOC->ODR &= ~GPIO_ODR_12;
			
			AllLedInvert(PushButtListnr(&button0));
			CaseLedInvert(PushButtListnr(&button1), 1);
			CaseLedInvert(PushButtListnr(&button2), 3);
			CaseLedInvert(PushButtListnr(&button3), 2);
			CaseLedInvert(PushButtListnr(&button4), 4);
		}
	}
}
