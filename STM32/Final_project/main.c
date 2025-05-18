#include "stm32f0xx.h"
#include <math.h>

#define MAX_DAC_VALUE 4095
#define BUFFER_SIZE 256
#define M_PI 3.14159265358979323846

void system_clock_init(void);
void gpio_init(void);
void dac_init(void);
void timer_init(void);
void dma_init(void);
void fill_sine_buffer(void);
//void fill_sine_buffer_circle(void);
//void fill_sine_buffer_complex(void);

static uint16_t dma_buffer[2][BUFFER_SIZE] __attribute__((aligned(4)));

void system_clock_init(void) {
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY));
	RCC->CFGR = 0;
	SystemCoreClockUpdate();
}

void gpio_init(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= (GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
}

void dac_init(void) {
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	DAC->CR |= DAC_CR_TEN1 | DAC_CR_TSEL1_0 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_2;
	DAC->CR |= DAC_CR_TEN2 | DAC_CR_TSEL2_0 | DAC_CR_TSEL2_1 | DAC_CR_TSEL2_2;
	DAC->CR |= DAC_CR_DMAEN1 | DAC_CR_DMAEN2;
	DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;
}

void timer_init(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->ARR = 8000000 / (BUFFER_SIZE * 1000) - 1;
	TIM6->PSC = 0;
	TIM6->CR2 |= TIM_CR2_MMS_1;
	TIM6->CR1 |= TIM_CR1_CEN;
}

void dma_init(void) {
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel3->CPAR = (uint32_t)&DAC->DHR12R1;
	DMA1_Channel3->CMAR = (uint32_t)dma_buffer[0];
	DMA1_Channel3->CNDTR = BUFFER_SIZE;
	DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_EN;
	DMA1_Channel4->CPAR = (uint32_t)&DAC->DHR12R2;
	DMA1_Channel4->CMAR = (uint32_t)dma_buffer[1];
	DMA1_Channel4->CNDTR = BUFFER_SIZE;
	DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_EN;
}

void fill_sine_buffer(void) {
	// figure 8 (2:1 frequency ratio)
	for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
		dma_buffer[0][i] = (uint16_t)((sin(2 * M_PI * 2.0f * i / BUFFER_SIZE) + 1) * (MAX_DAC_VALUE / 2));
		dma_buffer[1][i] = (uint16_t)((sin(2 * M_PI * 1.0f * i / BUFFER_SIZE + M_PI / 2) + 1) * (MAX_DAC_VALUE / 2));
	}
}

/*
void fill_sine_buffer_circle(void) {
	// circle (1:1 frequency ratio, 90-degree phase shift)
	for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
		dma_buffer[0][i] = (uint16_t)((sin(2 * M_PI * 1.0f * i / BUFFER_SIZE) + 1) * (MAX_DAC_VALUE / 2));
		dma_buffer[1][i] = (uint16_t)((sin(2 * M_PI * 1.0f * i / BUFFER_SIZE + M_PI / 2) + 1) * (MAX_DAC_VALUE / 2));
	}
}
*/

/*
void fill_sine_buffer_complex(void) {
	// complex figure (3:2 frequency ratio)
	for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
		dma_buffer[0][i] = (uint16_t)((sin(2 * M_PI * 3.0f * i / BUFFER_SIZE) + 1) * (MAX_DAC_VALUE / 2));
		dma_buffer[1][i] = (uint16_t)((sin(2 * M_PI * 2.0f * i / BUFFER_SIZE) + 1) * (MAX_DAC_VALUE / 2));
	}
}
*/

int main(void) {
	system_clock_init();
	gpio_init();
	dac_init();
	timer_init();
	dma_init();
	fill_sine_buffer();
	while (1) {
		// TODO
	}
}