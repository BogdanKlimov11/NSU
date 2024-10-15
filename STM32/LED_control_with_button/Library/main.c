#include "stm32f0xx.h"
#include "buttons.h"
#include "leds.h"

void init(void);

//int ndelay(uint32_t n,uint32_t* np);

void init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOC -> MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
	GPIOC -> MODER |= (GPIO_MODER_MODER6_0| GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0| GPIO_MODER_MODER9_0);

	GPIOC->ODR |= GPIO_ODR_6;

	GPIOA->MODER &= ~GPIO_MODER_MODER0;

	SystemCoreClockUpdate(); // System_Config(SystemCoreClock)
	SysTick->LOAD = SystemCoreClock / 1000 - 1; // 1 ms
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}

//can add "inline" at beginning

int main(void)
{
	init();
	
	struct buttons but;
	
	but.first = 1;
	but.second = 0;
	but.third = 0;
	but.four = 0;
	
	int light_delay = 0;
	
	while(1){
		/*if(((GPIO_IDR_0 & GPIOA->IDR) != 0) & (state == 0)){
			state = 1;
		}
		else if(((GPIO_IDR_0 & GPIOA->IDR) != 0) & (state == 1)){
			state = 0;
		}*/

		if(systick_flat_check()){
			light_delay = light_delay + 1;
			btn_filter();
		}

		//state = btn_state(); // tmp - nash state
		if(btn_pressed()){
			btn_set_pressed(0);
		}

		int s_witch = delay();
		if(s_witch == 1 && light_delay >= 200){
			light(btn_get_state(), &but);
			light_delay = 0;
		}
	}
}
