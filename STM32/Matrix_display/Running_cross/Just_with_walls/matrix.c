#include "matrix.h"

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
  SPI2->CR2 = SPI_CR2_DS; // | SPI_CR2
  SPI2->CR1 |= SPI_CR1_SPE;
}