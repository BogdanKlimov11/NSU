/* Sending values to two DAC channels */
#include "stm32f0xx.h"

void dac_init(void);
void init(void);

void init(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	SystemCoreClockUpdate();
}

// size of DAC = 12 bit
void dac_init(void) {
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	DAC->CR |= DAC_CR_EN1; // PA4 - yellow

	DAC->CR |= DAC_CR_EN2; // PA5 - blue
}

int main(void) {
	init();
	dac_init();

	while (1) {
		DAC->DHR12R1 = 2048;

		DAC->DHR12R2 = 0;
	}
}
