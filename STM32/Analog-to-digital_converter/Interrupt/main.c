#include "stm32f0xx.h"

static uint8_t ADC_array[256];
static int Matrix[8][8];
static uint8_t obrab = 0;
static uint8_t pos_first_half;
static uint8_t pos_second_half;
static uint8_t tmp_1 = 0;
static uint64_t sum = 0;
static uint8_t mat = 2;
static uint8_t curr_column = 0;
static uint16_t data = 0;

uint8_t systick_flag_check(void);

void init(void);
void adc_init(void);
void dma_init(void);
void matrix_init(void);
void matrix_update(void);
void matem(void);

void SPI2_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);

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
	SysTick->LOAD = SystemCoreClock / 10 - 1;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
}

void adc_init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER1;
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	if ((ADC1->CR & ADC_CR_ADEN))
	{
	ADC1->CR |= ADC_CR_ADDIS; 
	}
	
	while (ADC1->CR & ADC_CR_ADEN);
	
	ADC1->CR |= ADC_CR_ADCAL;
	while(ADC1->CR & ADC_CR_ADCAL);
	
	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR & ADC_ISR_ADRDY));
	ADC1->CFGR1 = ADC_CFGR1_RES_1 | ADC_CFGR1_OVRMOD | ADC_CFGR1_CONT | ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;
	
	ADC1->CHSELR |= ADC_CHSELR_CHSEL1; 
	
	ADC1->CR |= ADC_CR_ADSTART;
}

void dma_init(void){
// 	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
// 	ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
// 	DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR));
// 	DMA1_Channel1->CMAR = (uint32_t)(ADC_array);
// 	DMA1_Channel1->CNDTR = 128;
// 	DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_TEIE | DMA_CCR_TCIE  | DMA_CCR_CIRC;
// 	DMA1_Channel1->CCR &=~ DMA_CCR_DIR;
// 	DMA1_Channel1->CCR |= DMA_CCR_EN;
//	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel1->CCR &= ~DMA_CCR_MEM2MEM;
	DMA1_Channel1->CCR &= ~DMA_CCR_PL_0;
	DMA1_Channel1->CCR &= ~DMA_CCR_PL_1;
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;
	DMA1_Channel1->CCR |= DMA_CCR_MINC;
	DMA1_Channel1->CCR |= DMA_CCR_TCIE;
	DMA1_Channel1->CCR |= DMA_CCR_HTIE;
	DMA1_Channel1->CCR &= ~DMA_CCR_DIR;
	DMA1_Channel1->CNDTR = 256; 
	DMA1_Channel1->CPAR = (uint32_t)(&ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)(ADC_array);
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void matrix_init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	
	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	
	//GPIOB->AFR[0] |=  Fn << 4 * Pn; // Pn < 8
	//GPIOB->AFR[1] |=  Fn << 4 * (Pn - 8); // Pn > 7
	GPIOB->AFR[1] |= (0 << 4 * (13 - 8)) | (0 << 4 * (15 - 8));
	
	GPIOB->MODER &= ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER15);
	GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1;
	
	
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	
	SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_LSBFIRST | SPI_CR1_BR | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA;
	SPI2->CR2 = SPI_CR2_DS  | SPI_CR2_RXNEIE ;
	SPI2->CR1 |= SPI_CR1_SPE;
	
	NVIC_EnableIRQ(SPI2_IRQn);
	SPI2->DR=0;
	
	for (int i = 0; i < 8; i++){
		for (int j = 0; j < 8; j++){
			Matrix[i][j] = 0;
		}
	}
	//  Matrix[1][1]=1;
	//  Matrix[0][1]=1;
	//  Matrix[1][0]=1;
	//  Matrix[2][1]=1;
	//  Matrix[1][2]=1;
}

void matrix_update(void){
	if ((SPI2->SR & SPI_SR_BSY) == 0){
		GPIOA->BSRR = GPIO_BSRR_BS_8;
		
		data = 0;
		for(int i = 0; i < 8 ; i++)
		{
			if (Matrix[i][curr_column] == 1)
			data |= 1 << (i);
			
		}
		data |= 1 << (curr_column + 8);
		SPI2->DR = data; // selection of LEDs
		curr_column=curr_column + 1;
		if (curr_column == 8){
			curr_column = 0;
		}
		GPIOA->BSRR = GPIO_BSRR_BR_8;
	}
}

void matem(void){
	if (systick_flag_check()){
		if (mat == 0){
			sum = 0;
			for (int j = 0; j < 128; j++){
				sum += ADC_array[j];
			}
			tmp_1 = (uint8_t)(sum / 128);
			if(((uint16_t)tmp_1 <= 0x000000FF) && (tmp_1 > 0x000000DF))
				pos_first_half = 0b11111111;
			else
				if((tmp_1 <= 0x000000DF) && (tmp_1 > 0x000000BF))
					pos_first_half = 0b11111110;
				else
					if((tmp_1 <= 0x000000BF) && (tmp_1 > 0x0000009F))
						pos_first_half = 0b11111100;
					else
						if((tmp_1 <= 0x0000009F) && (tmp_1 > 0x0000007F))
							pos_first_half = 0b11111000;
						else
							if((tmp_1 <= 0x0000007F) && (tmp_1 > 0x0000005F))
								pos_first_half = 0b11110000;
							else
								if((tmp_1 <= 0x0000005F) && (tmp_1 > 0x0000003F))
									pos_first_half = 0b11100000;
								else
									if((tmp_1 <= 0x0000003F) && (tmp_1 > 0x0000001F))
										pos_first_half = 0b11000000;
									else
										pos_first_half = 0b10000000;
			
			for(int i = 6; i >= 0; i--){ 
				for(int j = 0; j < 8; j++){
					Matrix[j][i+1]=0;
					Matrix[j][i+1]=Matrix[j][i];
				}
			}
			for(int j = 0; j < 8; j++){
				if (pos_first_half&(1 << j)){
	 				Matrix[j][0] = 1;
				}
				else{ 
	  				Matrix[j][0] = 0;
				}
			}
			obrab = 0;
		}
		else{
			if(mat == 1){
				sum = 0;
				for (int j = 128; j < 256; j++){
					sum+=ADC_array[j];
				}
				tmp_1 = (uint8_t)(sum / 128);
				if(((uint16_t)tmp_1 <= 0x000000FF) && (tmp_1 > 0x000000DF))
					pos_second_half = 0b11111111;
				else
					if((tmp_1 <= 0x000000DF) && (tmp_1 > 0x000000BF))
						pos_second_half = 0b11111110;
					else
						if((tmp_1 <= 0x000000BF) && (tmp_1 > 0x0000009F))
							pos_second_half = 0b11111100;
						else
							if((tmp_1 <= 0x0000009F) && (tmp_1 > 0x0000007F))
								pos_second_half = 0b11111000;
							else
								if((tmp_1 <= 0x0000007F) && (tmp_1 > 0x0000005F))
									pos_second_half = 0b11110000;
								else
									if((tmp_1 <= 0x0000005F) && (tmp_1 > 0x0000003F))
										pos_second_half = 0b11100000;
									else
										if((tmp_1 <= 0x0000003F) && (tmp_1 > 0x0000001F))
											pos_second_half = 0b11000000;
										else
											pos_second_half = 0b10000000;
				for(int i = 6; i >= 0; i--){
					for(int j = 0; j < 8; j++){
						Matrix[j][i+1] = 0;
						Matrix[j][i+1] = Matrix[j][i];
					}
				}
				for (int j = 0; j < 8; j++ ){
					if(pos_first_half&(1 << j))
						Matrix[j][0] = 1;
					else 
						Matrix[j][0] = 0;
				}
				obrab = 0;
			}
		}
	}
}

void DMA1_Channel1_IRQHandler(void){
	if ((DMA1->ISR & DMA_ISR_TCIF1) != 0){
		DMA1->IFCR = DMA_IFCR_CTCIF1; 
		mat = 0;
		obrab = 1;
	}
	if ((DMA1->ISR & DMA_ISR_HTIF1) != 0){
		DMA1->IFCR = DMA_IFCR_CHTIF1; 
		mat = 1;
		obrab = 1;
	}
}

void SPI2_IRQHandler(void){
	if (SPI2->SR & SPI_SR_RXNE){
		matrix_update();
		uint16_t tmp = (uint16_t)SPI2->DR;
		tmp = 0;
	}
}

int main(void){
	init();
	adc_init();
	dma_init();
	matrix_init();
	while(1){ 
		if (obrab == 1){
			matem();
		}
	}
}
