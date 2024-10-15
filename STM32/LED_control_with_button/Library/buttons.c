#include "buttons.h"
#include "stm32f0xx.h"

static uint8_t btn_stable_curr_state = 0;
static uint8_t btn_stable_prev_state = 0;
static uint8_t _edge_detected = 0;
static uint8_t _unpressed = 0;
static uint32_t BTN_T = 5; // 5 ms

static uint8_t state = 0;
static uint8_t _pressed = 0;


uint8_t btn_pressed(){
 return _pressed;
}


void btn_set_pressed(uint8_t pr){
 _pressed=pr;
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


uint8_t btn_get_state(){
	return state;
}


uint8_t btn_filter(){ 
	//static uint8_t button_state = 0;
	static uint8_t _curr = 0, _prev = 0;
	static uint32_t _counter = 0;
	_curr = (GPIO_IDR_0 & GPIOA->IDR) ? 1 : 0;
	if(_prev != _curr){
		_prev = _curr;
		_counter = 0;
	}
	else{
		_counter++;
		if(_counter > BTN_T){
			btn_stable_curr_state = _curr;
			if(btn_stable_curr_state != btn_stable_prev_state){
				btn_stable_prev_state = btn_stable_curr_state;
				if(btn_stable_prev_state == 0){
					state = !state;
					_unpressed = 1;
				}
			}
		}
	}
	return btn_stable_curr_state;
}
