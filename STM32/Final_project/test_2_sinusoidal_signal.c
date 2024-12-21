/* Sending values of sinusoidal signal to two DAC channels */
#include "stm32f0xx.h"
#include <math.h>

#define MAX_DAC_VALUE 4095
#define BUFFER_SIZE 256
#define M_PI 3.14159265358979323846

void dac_init(void);
void init(void);
void fill_sine_buffer(void);

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

void fill_sine_buffer(void) {
	for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
		dma_buffer[i] = (uint32_t)((sin(2 * M_PI * i / BUFFER_SIZE) + 1) * (MAX_DAC_VALUE / 2));
	}
}

int main(void) {
	init();
	dac_init();
	fill_sine_buffer();

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
