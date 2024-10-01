#include <stm32f0xx.h>

// PC6 - red LED
// PC7 - blue LED
// PC8 - green LED
// PC9 - orange LED
// PA0 -button
// R |= M; set bit
// R &= ~M; reset bit

static void init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
	
	GPIOA->MODER &= ~GPIO_MODER_MODER0;
	
	GPIOC->MODER &= ~(GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	
	SystemCoreClockUpdate();
	SysTick->LOAD = SystemCoreClock / 2 - 1; // 1 sec
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}


static uint8_t systick_flag_check(void){
	return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1 : 0;
}


static uint8_t button_read(void){
	return (GPIOA->IDR & GPIO_IDR_0) ? 1 : 0;
}


static void led_red(uint8_t state){
	/*
	if(state){
		//GPIOC->ODR |= GPIO_ODR_6;
		GPIOC->BSRR = GPIO_BSRR_BS_6;
	}
	else{
		//GPIOC->ODR |= GPIO_ODR_6;
		GPIOC->BRR = GPIO_BRR_BR_6;
	}
	*/
	GPIOC->BSRR = (state) ? GPIO_BSRR_BS_6 : GPIO_BSRR_BR_6;
}


static void led_blue(uint8_t state){
	/*
	if(state){
		//GPIOC->ODR |= GPIO_ODR_7;
		GPIOC->BSRR = GPIO_BSRR_BS_7;
	}
	else{
		//GPIOC->ODR |= GPIO_ODR_7;
		GPIOC->BRR = GPIO_BRR_BR_7;
	}
	*/
	GPIOC->BSRR = (state) ? GPIO_BSRR_BS_7 : GPIO_BSRR_BR_7;
}


static void led_green(uint8_t state){
	/*
	if(state){
		//GPIOC->ODR |= GPIO_ODR_8;
		GPIOC->BSRR = GPIO_BSRR_BS_8;
	}
	else{
		//GPIOC->ODR |= GPIO_ODR_8;
		GPIOC->BRR = GPIO_BRR_BR_8;
	}
	*/
	GPIOC->BSRR = (state) ? GPIO_BSRR_BS_8 : GPIO_BSRR_BR_8;
}


static void led_orange(uint8_t state){
	/*
	if(state){
		//GPIOC->ODR |= GPIO_ODR_9;
		GPIOC->BSRR = GPIO_BSRR_BS_9;
	}
	else{
		//GPIOC->ODR |= GPIO_ODR_9;
		GPIOC->BRR = GPIO_BRR_BR_9;
	}
	*/
	GPIOC->BSRR = (state) ? GPIO_BSRR_BS_9 : GPIO_BSRR_BR_9;
}


int main(void){
 	init();
	int state;
	int current;
	while(1){
		state = button_read();
		if(state){
			led_red(1);
			while(!systick_flag_check()){
				current = button_read();
				if(current != state){
					state = current;
					break;
				}
			}
			led_red(0);
			led_green(1);
			while(!systick_flag_check()){
				current = button_read();
				if(current != state){
					state = current;
					break;
				}
			}
			led_green(0);
			led_blue(1);
			while(!systick_flag_check()){
				current = button_read();
				if(current != state){
					state = current;
					break;
				}
			}
			led_blue(0);
			led_orange(1);
			while(!systick_flag_check()){
				current = button_read();
				if(current != state){
					state = current;
					break;
				}
			}
			led_orange(0);
		}
		else{
			led_red(1);
			while(!systick_flag_check()){
				current = button_read();
				if(current != state){
					state = current;
					break;
				}
			}
			led_red(0);
			led_orange(1);
			while(!systick_flag_check()){
				current = button_read();
				if(current != state){
					state = current;
					break;
				}
			}
			led_orange(0);
			led_blue(1);
			while(!systick_flag_check()){
				current = button_read();
				if(current != state){
					state = current;
					break;
				}
			}
			led_blue(0);
			led_green(1);
			while(!systick_flag_check()){
				current = button_read();
				if(current != state){
					state = current;
					break;
				}
			}
			led_green(0);
		}
	}
}
