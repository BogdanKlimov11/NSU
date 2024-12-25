#include "stm32f0xx.h"

static uint8_t ADC_array[256];
static uint16_t pos = 0;
static uint16_t sum = 0;
static uint16_t tmp_1 = 0;

uint8_t systick_flag_check(void);

void init(void);
void matrix_init(void);
void adc_init(void);

void SPI2_IRQHandler(void);

uint8_t systick_flag_check(void){
	return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1 : 0;
}

void init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;

	GPIOA->MODER &= ~(GPIO_MODER_MODER0|GPIO_MODER_MODER4|GPIO_MODER_MODER5);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4|GPIO_PUPDR_PUPDR5);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1|GPIO_PUPDR_PUPDR5_1;
	GPIOC->MODER |= GPIO_MODER_MODER12_0;
	GPIOA->MODER |= GPIO_MODER_MODER15_0;

	GPIOC->MODER &= ~(GPIO_MODER_MODER6|GPIO_MODER_MODER7|GPIO_MODER_MODER8|GPIO_MODER_MODER9);
	GPIOC->MODER |= GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0;

	SystemCoreClockUpdate();
	SysTick->LOAD = SystemCoreClock / 1000 - 1; // 1 ms
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
}

void matrix_init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;

	GPIOB->MODER &= ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER15);
	GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1; // alternative func
	//GPIOB->AFR[0] |=  Fn << 4 * Pn; // Pn < 8 Fn - number of alt. func.
	//GPIOB->AFR[1] |=  Fn << 4 * (Pn - 8); // Pn > 7 // -8 to < 32

	GPIOB->AFR[1] |= (0 << 4 * (13 - 8)) | (0 << 4 * (15 - 8));

	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI |/* SPI_CR1_LSBFIRST |*/ SPI_CR1_BR | SPI_CR1_MSTR /*| SPI_CR1_CPOL | SPI_CR1_CPHA*/;
	SPI2->CR2 = SPI_CR2_DS |SPI_CR2_RXNEIE;
	SPI2->CR1 |= SPI_CR1_SPE;

	//NVIC_SetPriority(SPI2_IRQn);
	NVIC_EnableIRQ(SPI2_IRQn);
	SPI2->DR = 0;
}

void adc_init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER1;
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	if ((ADC1->CR & ADC_CR_ADEN)){ /* (1) */
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}

	//ADC1->CR |= ADC_CR_ADDIS;
	while (ADC1->CR & ADC_CR_ADEN);

	ADC1->CR |= ADC_CR_ADCAL;
	while(ADC1->CR & ADC_CR_ADCAL);

	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR & ADC_ISR_ADRDY));
	
	ADC1->CFGR1 = ADC_CFGR1_RES_1 | ADC_CFGR1_OVRMOD | ADC_CFGR1_CONT; // 8 bit
	ADC1->CHSELR |= ADC_CHSELR_CHSEL1;
	
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)ADC_array;
	DMA1_Channel1->CNDTR = 256; 
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC;
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	
	ADC1->CR |= ADC_CR_ADSTART;
}

void SPI2_IRQHandler(void){
	for(int j = 0; j < 256; j++){
		sum+=ADC_array[j];
	}
	tmp_1 = (uint16_t)(sum / 256);
	sum = 0;
	if((tmp_1 <= 0x000000FF) && (tmp_1 > 0x000000DF))
		pos = 0b11111111;
	else
		if((tmp_1 <= 0x000000DF) && (tmp_1 > 0x000000BF))
			pos = 0b11111110;
		else
			if((tmp_1 <= 0x000000BF) && (tmp_1 > 0x0000009F))
				pos = 0b11111100;
			else
				if((tmp_1 <= 0x0000009F) && (tmp_1 > 0x0000007F))
					pos = 0b11111000;
				else
					if((tmp_1 <= 0x0000007F) && (tmp_1 > 0x0000005F))
						pos = 0b11110000;
					else
						if((tmp_1 <= 0x0000005F) && (tmp_1 > 0x0000003F))
							pos = 0b11100000;
						else
							if((tmp_1 <= 0x0000003F) && (tmp_1 > 0x0000001F))
								pos = 0b11000000;
							else
								pos = 0b10000000;

	SPI2->DR = pos | (1 << 8);
	GPIOA->BSRR = GPIO_BSRR_BS_8;
	GPIOA->BRR = GPIO_BRR_BR_8;
	uint16_t tmp = (uint16_t)SPI2->DR;
	tmp = 0;
}

int main(void){
	init();
	adc_init();
	matrix_init();
	while(1){
		//TODO
	}
}
