#include <stm32f0xx.h>

// PC6 - red LED
// PA0 -button
// R |= M; set bit
// R &= ~M; reset bit

static void init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER &= ~GPIO_MODER_MODER6;
}


static void led(uint8_t state){
	if(state){
		GPIOC->MODER |= GPIO_MODER_MODER6_0;
		GPIOC->ODR |= GPIO_ODR_6;
	}
	else{
		GPIOC->MODER &= ~GPIO_MODER_MODER6;
		GPIOC->ODR |= GPIO_ODR_6;
	}
}


int main(void){
	init();
	while(1){
		for(int i = 0; i<=200000; i++){
			led((uint8_t)1);
		}
		for(int i = 0; i<=200000; i++){
			led((uint8_t)0);
		}
	}
}
