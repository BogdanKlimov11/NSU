#include "button.h"
#include "stm32f0xx.h"

static uint16_t posStroka = 0b0000001000000000;
static uint16_t posStolb = 0b0000000000000010;
static uint16_t posPalkaStroka = 0;
static uint16_t posPalkaStolb = 0;
static uint16_t posUpDotStroka = 0;
static uint16_t posUpDotStolb = 0;
static uint16_t posDownDotStroka = 0;
static uint16_t posDownDotStolb = 0;
static int HandleCount = 0;
//static uint16_t posFinStroka = 0;
//static uint16_t posFinStolb = 0;
static uint16_t pos = 0b0;

static uint8_t ADC_array[128];
static uint16_t sum = 0;
static uint16_t tmp_1 = 0;

void init(void);
void matrix_init(void);
void adc_init(void);

uint8_t systick_flag_check(void);
void SysTick_Handler(void);

uint8_t button_0_read(void); 
uint8_t button_1_read(void); 
uint8_t button_2_read(void);
uint8_t button_3_read(void);
uint8_t button_4_read(void);

void SPI2_IRQHandler(void);
void CaseLedInvert(uint8_t state, int8_t num);
void AllLedInvert(uint8_t state);

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
	DMA1_Channel1->CPAR = &(ADC1->DR);
	DMA1_Channel1->CMAR = ADC_array;
	DMA1_Channel1->CNDTR = 128; 
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC;
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	
	ADC1->CR |= ADC_CR_ADSTART;
}

void SPI2_IRQHandler(void){
//	if (SPI2->SR & SPI_SR_RXNE){
//			if(HandleCount%3  == 1){
//			posFinStolb = posUpDotStolb;
//			posFinStroka = posUpDotStroka;
//		}
//		if(HandleCount%3  == 2){
//			posFinStolb = posDownDotStolb;
//			posFinStroka = posDownDotStroka;
//		}
//		if(HandleCount%3  == 0){
//			posFinStolb = posPalkaStolb;
//			posFinStroka = posPalkaStroka;
//		}
//		
//		SPI2->DR = posFinStroka | posFinStolb; 
//		GPIOA->BSRR = GPIO_BSRR_BS_8;
//		GPIOA->BRR = GPIO_BRR_BR_8;
//		volatile uint16_t tmp = SPI2->DR;
//	}

	for (int j = 0; j < 128; j++){
		sum+=ADC_array[j];
	}
	tmp_1 = (uint16_t)(sum/128);
	sum = 0;
	if((tmp_1 <= 0x000000FF) && (tmp_1 >0x000000DF))
			pos = 0b11111111;
		else if((tmp_1 <= 0x000000DF) && (tmp_1 >0x000000BF))
			pos = 0b11111110;
		else if((tmp_1 <= 0x000000BF) && (tmp_1 > 0x0000009F))
			pos = 0b11111100;
		else if((tmp_1 <= 0x0000009F) && (tmp_1 >0x0000007F))
			pos = 0b11111000;
		else if((tmp_1 <= 0x0000007F) && (tmp_1 >0x0000005F))
			pos = 0b11110000;
		else if((tmp_1 <= 0x0000005F) && (tmp_1 >0x0000003F))
			pos = 0b11100000;
		else if((tmp_1 <= 0x0000003F) && (tmp_1 >0x0000001F))
			pos = 0b11000000;
		else
			pos = 0b10000000;

	SPI2->DR = pos | 0b0000010000000000;
	GPIOA->BSRR = GPIO_BSRR_BS_8;
	GPIOA->BRR = GPIO_BRR_BR_8;
	volatile uint16_t tmp = SPI2->DR;
}

uint8_t systick_flag_check(void){
  return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ? 1 : 0;
}

void CaseLedInvert(uint8_t state, int8_t num){
	if (state){
		switch (num){
			case 1:
				GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_6) == GPIO_ODR_6) ? GPIO_BSRR_BR_6 : GPIO_BSRR_BS_6;
				if(posStolb != 0b0000000000000010){
					posStolb = (uint16_t)(posStolb >> 1);
				}
				posPalkaStolb = posStolb;
				posDownDotStolb = posStolb >> 1;
				posUpDotStolb = posStolb << 1;
				break;
			case 2:
				GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_9) == GPIO_ODR_9) ? GPIO_BSRR_BR_9 : GPIO_BSRR_BS_9;
				if(posStroka != 0b0100000000000000){
					posStroka = (uint16_t)(posStroka << 1);
				}
				posPalkaStroka = (posStroka) | (posStroka << 1) | (posStroka >> 1);
				posUpDotStroka = posStroka;
				posDownDotStroka = posStroka;
				break;
			case 3:
				GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_7) == GPIO_ODR_7) ? GPIO_BSRR_BR_7 : GPIO_BSRR_BS_7;
				if(posStolb != 0b0000000001000000){
					posStolb = (uint16_t)(posStolb << 1);
				}
				posPalkaStolb = posStolb;
				posDownDotStolb = posStolb >> 1;
				posUpDotStolb = posStolb << 1;
				break;
			case 4:
				GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_8) == GPIO_ODR_8) ? GPIO_BSRR_BR_8 : GPIO_BSRR_BS_8;
				if(posStroka != 0b0000001000000000){
					posStroka = (uint16_t)(posStroka >> 1);
				}
				posPalkaStroka = (posStroka) | (posStroka << 1) | (posStroka >> 1);
				posUpDotStroka = posStroka;
				posDownDotStroka = posStroka;
				break;
			default:
				break;
		}
	}
}

uint8_t button_0_read(void){
	return (GPIO_IDR_0 & GPIOA->IDR) ? 1 : 0;
}

uint8_t button_1_read(void){
  //GPIOA->ODR |= GPIO_ODR_15;
  return (GPIO_IDR_4 & GPIOA->IDR) ? 1 : 0;
  //GPIOA->ODR &= ~GPIO_ODR_15;
  //return a;
}

uint8_t button_2_read(void){
  //GPIOA->ODR |= GPIO_ODR_15;
  return (GPIO_IDR_5 & GPIOA->IDR) ? 1 : 0;
  //GPIOA->ODR &= ~GPIO_ODR_15;
  //return a;
}

uint8_t button_3_read(void){
  //GPIOC->ODR |= GPIO_ODR_12;
  return (GPIO_IDR_4 & GPIOA->IDR) ? 1 : 0;
  //GPIOC->ODR &= ~GPIO_ODR_12;
  //return a;
}

uint8_t button_4_read(void){
  //GPIOC->ODR |= GPIO_ODR_12;
  return (GPIO_IDR_5 & GPIOA->IDR) ? 1 : 0;
  //GPIOC->ODR &= ~GPIO_ODR_12;
  //return a;
}


void AllLedInvert(uint8_t state){
	CaseLedInvert(state, 1);
	CaseLedInvert(state, 2);
	CaseLedInvert(state, 3);
	CaseLedInvert(state, 4);
}

static ConfButton button1; 
static ConfButton button2;
static ConfButton button3;
static ConfButton button4;

void SysTick_Handler(void){
	posPalkaStolb = posStolb;
	posDownDotStolb = posStolb >> 1;
	posUpDotStolb = posStolb << 1;
	posPalkaStroka = (posStroka) | (posStroka << 1) | (posStroka >> 1);
	posUpDotStroka = posStroka;
	posDownDotStroka = posStroka;
	HandleCount++;

	GPIOA->ODR |= GPIO_ODR_15;
	HandlerButton(&button1);
	HandlerButton(&button2);
	GPIOA->ODR &= ~GPIO_ODR_15;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	GPIOC->ODR |= GPIO_ODR_12;
	HandlerButton(&button3);
	HandlerButton(&button4);
	GPIOC->ODR &= ~GPIO_ODR_12;

	CaseLedInvert(PushButtListnr(&button1), 1);
	CaseLedInvert(PushButtListnr(&button2), 3);
	CaseLedInvert(PushButtListnr(&button3), 2);
	CaseLedInvert(PushButtListnr(&button4), 4);
}

int main(void){
	init();
	adc_init();
	matrix_init();

	button1 = InitButton(button_1_read);
	button2 = InitButton(button_2_read);
	button3 = InitButton(button_3_read);
	button4 = InitButton(button_4_read);
	while(1){}
}
