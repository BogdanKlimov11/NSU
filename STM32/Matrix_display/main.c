#include "stm32f0xx.h"

void init(void);

void matrix_init(void);

void init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	
	GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR5_1;
	GPIOC->MODER |= GPIO_MODER_MODER12_0;
	GPIOA->MODER |= GPIO_MODER_MODER15_0;
	
	GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	
	SystemCoreClockUpdate();
	SysTick->LOAD = SystemCoreClock / 1000 - 1; // 1 ms
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
}

// PB15__SPI2_MOSI
// PB13__SPI2_SCK
// PA8__GPO_LE
void matrix_init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	
	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	
	GPIOB->MODER &= ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER15);
	GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1;
	//GPIOB->AFR[0] |=  Fn << 4 * Pn; // Pn < 8
	//GPIOB->AFR[1] |=  Fn << 4 * (Pn - 8); // Pn > 7
	GPIOB->AFR[1] |= (0 << 4 * (13 - 8)) | (0 << 4 * (15 - 8));
	
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	
	SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI |/* SPI_CR1_LSBFIRST |*/ SPI_CR1_BR | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA;
	SPI2->CR2 = SPI_CR2_DS;
	SPI2->CR1 |= SPI_CR1_SPE;
}

int main(void){
	init();
	matrix_init();
	SPI2->DR = 0x80ff; // selection of diodes
	while (SPI2->SR & SPI_SR_BSY);
		GPIOA->BSRR = GPIO_BSRR_BS_8;
	while(1){
		// TODO
	}
}
