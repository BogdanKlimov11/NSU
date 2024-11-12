#include "cross.h"

void paint(uint8_t start_x, uint8_t start_y, uint8_t shift){
	GPIOA->BSRR = GPIO_BSRR_BR_8;
	SPI2->DR = (uint32_t)(start_y << 8) | (1 << start_x);
	while(SPI2->SR & SPI_SR_BSY);
	GPIOA->BSRR = GPIO_BSRR_BS_8;
	
	GPIOA->BSRR = GPIO_BSRR_BR_8;
	SPI2->DR = (uint32_t)((start_y - shift) << 8) | (1 << (start_x - 1));
	while(SPI2->SR & SPI_SR_BSY);
	GPIOA->BSRR = GPIO_BSRR_BS_8;
	
	GPIOA->BSRR = GPIO_BSRR_BR_8;
	SPI2->DR = (uint32_t)((start_y - shift) << 8) | (1 << (start_x + 1));
	while(SPI2->SR & SPI_SR_BSY);
	GPIOA->BSRR = GPIO_BSRR_BS_8;
}
