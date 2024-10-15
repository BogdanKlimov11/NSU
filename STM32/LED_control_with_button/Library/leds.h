#ifndef _LEDS_H_

#define _LEDS_H_

#include "stm32f0xx.h"
#include "buttons.h"

void led(uint8_t state, struct buttons* buttons);
int delay(void);
uint8_t systick_flat_check(void);
void light(uint8_t, struct buttons*);

#endif /* _LEDS_H_ */
