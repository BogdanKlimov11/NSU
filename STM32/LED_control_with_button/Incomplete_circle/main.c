#include "stm32f0xx.h"

struct buttons inf_collection(struct buttons* but);
void init(void);
int delay(void);
void led(uint8_t state, struct buttons* buttons);
uint8_t btn_pressed(void);
uint8_t btn_unpressed(void);
uint8_t btn_edge_detected(void);
uint8_t btn_state(void);

//int ndelay(uint32_t n,uint32_t* np);

struct buttons{
	int first;
	int second;
	int third;
	int four;
};


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


static uint8_t systick_flat_check(void){
	return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1: 0;
}


int delay(void){
	if (systick_flat_check() == 1){
		return(1);
	}
	else{
		return(0);
	}
}


static void light(uint8_t state, struct buttons* buttons){
	if (state){
		if (buttons->first == 1){
			//GPIOC->ODR &= ~GPIO_ODR_6;
			GPIOC->BRR = GPIO_BRR_BR_6;
			//GPIOC->ODR |= GPIO_ODR_8;
			GPIOC->BSRR = GPIO_BSRR_BS_8;
			buttons->first = 0;
			buttons->second = 1;
			return;
			
			//GPIOC->BSRR = state ? CPIO_BSRR_BS_6 : GPIO_BSRR_BR_6; // for general purpose ports only
    		}
		if (buttons->second == 1){
			//GPIOC->ODR &= ~GPIO_ODR_8;
			GPIOC->BRR = GPIO_BRR_BR_8;
			//GPIOC->ODR |= GPIO_ODR_7; 
			GPIOC->BSRR = GPIO_BSRR_BS_7;
			buttons->second = 0;
			buttons->third = 1;
			return;
		}
		if (buttons->third == 1){
			//GPIOC->ODR &= ~GPIO_ODR_7;
			GPIOC->BRR = GPIO_BRR_BR_7;
			//GPIOC->ODR |= GPIO_ODR_9; 
			GPIOC->BSRR = GPIO_BSRR_BS_9;
			buttons->third = 0;
			buttons->four = 1;
			return;
		}
		if (buttons->four == 1){
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
		if (buttons->first == 1){
			//GPIOC->ODR &= ~GPIO_ODR_6;
			GPIOC->BRR = GPIO_BRR_BR_6;
			//GPIOC->ODR |= GPIO_ODR_9; 
			GPIOC->BSRR = GPIO_BSRR_BS_9;
			buttons->first = 0;
			buttons->four = 1;
			return;
		}
		if (buttons->second == 1){
			//GPIOC->ODR &= ~GPIO_ODR_8;
			GPIOC->BRR = GPIO_BRR_BR_8;
			//GPIOC->ODR |= GPIO_ODR_6; 
			GPIOC->BSRR = GPIO_BSRR_BS_6;
			buttons->second = 0;
			buttons->first = 1;
			return;
		}
		if (buttons->third == 1){
			//GPIOC->ODR &= ~GPIO_ODR_7;
			GPIOC->BRR = GPIO_BRR_BR_7;
			//GPIOC->ODR |= GPIO_ODR_8; 
			GPIOC->BSRR = GPIO_BSRR_BS_8;
			buttons->third = 0;
			buttons->second = 1;
			return;
		}
		if (buttons->four == 1){
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


static uint8_t btn_stable_curr_state = 0;
static uint8_t btn_stable_prev_state = 0;
static uint8_t _edge_detected = 0;
static uint8_t _pressed = 0;
static uint8_t _unpressed = 0;
static uint32_t BTN_T = 5; // 5 ms
static uint8_t state = 0;


static uint8_t btn_filter(){
	//static uint8_t button_state = 0;
	static uint8_t _curr = 0, _prev = 0;
	static uint32_t _counter = 0;
	_curr = (GPIO_IDR_0 & GPIOA->IDR) ? 1 : 0;
	if (_prev != _curr){
		_prev = _curr;
		_counter = 0;
	}
	else{
		_counter++;
		if (_counter > BTN_T){
			btn_stable_curr_state = _curr;
			if (btn_stable_curr_state != btn_stable_prev_state){
				btn_stable_prev_state = btn_stable_curr_state;
				//_edge_detected = 1;
				if (btn_stable_prev_state == 0){
					state = !state;
					_unpressed = 1;
				}
				/*if(btn_stable_curr_state){
					_pressed = 1;
				}
				else{
					_unpressed = 1;
				}*/
			}
		}
	}
	return btn_stable_curr_state;
}

//can add "inline" at beginning
uint8_t btn_pressed(){
	return _pressed;
}


uint8_t btn_unpressed(){
	return _unpressed;
}


uint8_t btn_edge_detected(){
	return _edge_detected;
}


uint8_t btn_state(){
	return btn_stable_curr_state;
}


int main(void){
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
		
		if (systick_flat_check()){
			light_delay = light_delay + 1;
			btn_filter();
		}
		
		//state = btn_state(); // tmp - nash state
		if (btn_pressed()){
			_pressed = 0;
		}
		
		int s_witch = delay();
		if (s_witch == 1 && light_delay >= 200){
			light(state, &but);
			light_delay = 0;
		}
	}
}
