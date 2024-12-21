#include "stm32f0xx.h"
#include <math.h>

#define MAX_DAC_VALUE 4095
#define BUFFER_SIZE 256
#define PI 3.14159265359

void init(void);
void dac_init(void);
void dma_init(void);
void fill_lissajous_buffers(void);

static uint32_t buffer1[BUFFER_SIZE]; // Buffer for DAC channel 1
static uint32_t buffer2[BUFFER_SIZE]; // Buffer for DAC channel 2

void init(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	SystemCoreClockUpdate();
}

void dac_init(void) {
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;
	DAC->CR |= DAC_CR_DMAEN1 | DAC_CR_DMAEN2;
}

void dma_init(void) {
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable clock for DMA1

	// Configure DMA1_Channel3 for DAC1
	DMA1_Channel3->CPAR = (uint32_t)&DAC->DHR12R1; // Set peripheral address to DAC1 data register
	DMA1_Channel3->CMAR = (uint32_t)buffer1;       // Set memory address to buffer1
	DMA1_Channel3->CNDTR = BUFFER_SIZE;            // Set number of data to transfer
	DMA1_Channel3->CCR = DMA_CCR_MINC              // Enable memory address increment
											| DMA_CCR_CIRC             // Enable circular mode
											| DMA_CCR_DIR              // Read from memory
											| DMA_CCR_PSIZE_0          // Set peripheral data size to 16 bits
											| DMA_CCR_MSIZE_0          // Set memory data size to 16 bits
											| DMA_CCR_PL_1;            // Set high priority

	// Configure DMA1_Channel4 for DAC2
	DMA1_Channel4->CPAR = (uint32_t)&DAC->DHR12R2; // Set peripheral address to DAC2 data register
	DMA1_Channel4->CMAR = (uint32_t)buffer2;       // Set memory address to buffer2
	DMA1_Channel4->CNDTR = BUFFER_SIZE;            // Set number of data to transfer
	DMA1_Channel4->CCR = DMA_CCR_MINC              // Enable memory address increment
											| DMA_CCR_CIRC             // Enable circular mode
											| DMA_CCR_DIR              // Read from memory
											| DMA_CCR_PSIZE_0          // Set peripheral data size to 16 bits
											| DMA_CCR_MSIZE_0          // Set memory data size to 16 bits
											| DMA_CCR_PL_1;            // Set high priority

	// Enable DMA channels
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void fill_lissajous_buffers(void) {
	double freq1 = (double)1.0f;  // Frequency for channel 1
	double freq2 = (double)2.0f;  // Frequency for channel 2
	double phase = PI / 2; // Phase shift between signals

	for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
		double t = (double)i / BUFFER_SIZE; // Time fraction
		buffer1[i] = (uint32_t)((sin(2 * PI * freq1 * t) + (double)1.0f) * (MAX_DAC_VALUE / 2));
		buffer2[i] = (uint32_t)((sin(2 * PI * freq2 * t + phase) + (double)1.0f) * (MAX_DAC_VALUE / 2));
	}
}

int main(void) {
	init();
	dac_init();
	fill_lissajous_buffers();
	dma_init();

	while (1) {
		//TODO
	}
}
