/* Sending values of square signal to two DAC channels */
#include "stm32f0xx.h"

#define MAX_DAC_VALUE 4095
#define BUFFER_SIZE 256

void dac_init(void);
void init(void);
void fill_square_buffer(void);

static uint32_t dma_buffer[BUFFER_SIZE];
static uint32_t index = 0;

void init(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	SystemCoreClockUpdate();
}

void dac_init(void) {
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	DAC->CR |= DAC_CR_EN1;
	DAC->CR |= DAC_CR_EN2;
}

void fill_square_buffer(void) {
	for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
		dma_buffer[i] = (i % 2 == 0) ? 0 : MAX_DAC_VALUE;
	}
}

int main(void) {
	init();
	dac_init();
	fill_square_buffer();

	while (1) {
		DAC->DHR12R1 = dma_buffer[index];
		DAC->DHR12R2 = dma_buffer[index];

		index++;
		if (index >= BUFFER_SIZE) {
			index = 0;
		}

		for (volatile int i = 0; i < 1000; i++);
	}
}
