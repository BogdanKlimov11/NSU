#ifndef _BUTTONS_H_

#define _BUTTONS_H_

#include "stm32f0xx.h"

uint8_t btn_filter(void);
uint8_t btn_state(void);
uint8_t btn_pressed(void);
uint8_t btn_unpressed(void);
uint8_t btn_edge_detected(void);

void btn_set_pressed(uint8_t);
uint8_t btn_get_state(void);

struct buttons inf_collection(struct buttons* but);

struct buttons{
	int first;
	int second;
	int third;
	int four;
};

#endif /* _BUTTON_H_ */
