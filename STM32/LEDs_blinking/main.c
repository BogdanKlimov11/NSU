#include <stm32f0xx.h>

// PC6 - red LED
// PC7 - blue LED
// PC8 - green LED
// PC9 - orange LED
// PA0 -button
// R |= M; set bit
// R &= ~M; reset bit

static void init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER &= ~GPIO_MODER_MODER6;
	GPIOC->MODER &= ~GPIO_MODER_MODER7;
	GPIOC->MODER &= ~GPIO_MODER_MODER8;
	GPIOC->MODER &= ~GPIO_MODER_MODER9;
}


static void led_6(uint8_t state){
	if(state){
		GPIOC->MODER |= GPIO_MODER_MODER6_0;
		GPIOC->ODR |= GPIO_ODR_6;
	}
	else{
		GPIOC->MODER &= ~GPIO_MODER_MODER6;
		GPIOC->ODR |= GPIO_ODR_6;
	}
}


static void led_7(uint8_t state){
	if(state){
		GPIOC->MODER |= GPIO_MODER_MODER7_0;
		GPIOC->ODR |= GPIO_ODR_7;
	}
	else{
		GPIOC->MODER &= ~GPIO_MODER_MODER7;
		GPIOC->ODR |= GPIO_ODR_7;
	}
}


static void led_8(uint8_t state){
	if(state){
		GPIOC->MODER |= GPIO_MODER_MODER8_0;
		GPIOC->ODR |= GPIO_ODR_8;
	}
	else{
		GPIOC->MODER &= ~GPIO_MODER_MODER8;
		GPIOC->ODR |= GPIO_ODR_8;
	}
}


static void led_9(uint8_t state){
	if(state){
		GPIOC->MODER |= GPIO_MODER_MODER9_0;
		GPIOC->ODR |= GPIO_ODR_9;
	}
	else{
		GPIOC->MODER &= ~GPIO_MODER_MODER9;
		GPIOC->ODR |= GPIO_ODR_9;
	}
}


int main(void){
	init();
	while(1){
		for(int i = 0; i<=200000; i++){
			led_6((uint8_t)1);
			led_7((uint8_t)0);
			led_8((uint8_t)0);
			led_9((uint8_t)0);
		}
		for(int i = 0; i<=200000; i++){
			led_6((uint8_t)0);
			led_7((uint8_t)0);
			led_8((uint8_t)0);
			led_9((uint8_t)1);
		}
		for(int i = 0; i<=200000; i++){
			led_6((uint8_t)0);
			led_7((uint8_t)1);
			led_8((uint8_t)0);
			led_9((uint8_t)0);
		}
		for(int i = 0; i<=200000; i++){
			led_6((uint8_t)0);
			led_7((uint8_t)0);
			led_8((uint8_t)1);
			led_9((uint8_t)0);
		}
	}
}
